"""One-action-per-decision Claude client for the camera-only Fetch task."""
from __future__ import annotations

import base64
import json
import os
import re
import time
import uuid

import httpx

from agent.fetch_tools import ACTION_PATH, FETCH_TOOLS
from agent.engine_client import EngineClient
from agent.prompts import FETCH_SYSTEM_PROMPT
from evaluation.fetch_logger import FetchLogger


def _image_block(payload: bytes) -> dict:
    return {"type": "image", "source": {"type": "base64", "media_type": "image/jpeg",
            "data": base64.b64encode(payload).decode("ascii")}}


def _status_text(command: str, state: dict, last_result: dict | None,
                 step: int = 0, history: list[dict] | None = None) -> str:
    recent = (history or [])[-5:]
    summaries = [f"{item['action']}: {json.dumps(item['result'], separators=(',', ':'))}"
                 for item in recent]
    return (f"Command: {command}\nStep: {step}\nState: {state.get('state')}\n"
            f"Robot pose: x={state.get('x')}, y={state.get('y')}, heading_deg={state.get('heading_deg')}\n"
            f"Holding: {state.get('holding')}\nCollision: {state.get('collided')}\n"
            f"Retries: {state.get('retry_count')}\nLast action result: {last_result or 'none'}\n"
            f"Last five actions: {' | '.join(summaries) if summaries else 'none'}")


def _keep_latest_image(messages: list[dict]) -> None:
    """Keep the newest camera frame in the conversation and remove older pixels."""
    latest = None
    for message_index, message in enumerate(messages):
        content = message.get("content", [])
        if isinstance(content, list):
            for block_index, block in enumerate(content):
                block_type = block.get("type") if isinstance(block, dict) else getattr(block, "type", None)
                if block_type == "image":
                    latest = (message_index, block_index)
    for message_index, message in enumerate(messages):
        content = message.get("content", [])
        if not isinstance(content, list):
            continue
        message["content"] = [
            block for block_index, block in enumerate(content)
            if (block.get("type") if isinstance(block, dict) else getattr(block, "type", None)) != "image"
            or (message_index, block_index) == latest]


def _groq_messages(messages: list[dict]) -> list[dict]:
    converted = [{"role": "system", "content": FETCH_SYSTEM_PROMPT}]
    for message in messages:
        role = message["role"]
        content = message.get("content", [])
        if not isinstance(content, list):
            converted.append({"role": role, "content": content})
            continue
        text_parts = []
        images = []
        tool_calls = []
        tool_results = []
        for block in content:
            kind = block.get("type") if isinstance(block, dict) else getattr(block, "type", None)
            if kind == "text":
                text_parts.append(block.get("text", "") if isinstance(block, dict) else block.text)
            elif kind == "image":
                source = block["source"]
                images.append({"type": "image_url", "image_url": {
                    "url": f"data:{source['media_type']};base64,{source['data']}"}})
            elif kind == "tool_use":
                item = block if isinstance(block, dict) else {
                    "id": block.id, "name": block.name, "input": block.input}
                tool_calls.append({"id": item["id"], "type": "function", "function": {
                    "name": item["name"], "arguments": json.dumps(item["input"])}})
            elif kind == "tool_result":
                tool_results.append({"role": "tool", "tool_call_id": block["tool_use_id"],
                                     "content": str(block.get("content", ""))})
        if role == "assistant" and tool_calls:
            entry = {"role": "assistant", "tool_calls": tool_calls}
            if text_parts:
                entry["content"] = "\n".join(text_parts)
            converted.append(entry)
        elif role == "user":
            converted.extend(tool_results)
            parts = [{"type": "text", "text": "\n".join(text_parts)}] if text_parts else []
            converted.append({"role": "user", "content": parts + images})
        elif text_parts:
            converted.append({"role": role, "content": "\n".join(text_parts)})
    return converted


def _groq_payload(messages: list[dict], model: str) -> dict:
    schema = {"type": "object", "properties": {
        "action": {"type": "string", "enum": [tool["name"] for tool in FETCH_TOOLS]},
        "distance_m": {"type": ["number", "null"], "description": "Move distance; null unless action is move."},
        "degrees": {"type": ["number", "null"], "description": "Turn angle; null unless action is turn."},
        "what_i_see": {"type": "string", "description": "Brief observation grounded in the camera image."},
        "why": {"type": "string", "description": "Reason for this one action."}},
        "required": ["action", "distance_m", "degrees", "what_i_see", "why"],
        "additionalProperties": False}
    payload = {"model": model, "messages": _groq_messages(messages),
               "response_format": {"type": "json_schema", "json_schema": {
                   "name": "fetch_action", "strict": True, "schema": schema}},
               "temperature": 0}
    if model.startswith("qwen/"):
        payload.update({"reasoning_effort": "none", "reasoning_format": "hidden"})
    return payload


def _groq_decision(messages: list[dict], model: str, api_key: str) -> tuple[dict, list, int, int]:
    payload = _groq_payload(messages, model)
    response = None
    for attempt in range(5):
        response = httpx.post("https://api.groq.com/openai/v1/chat/completions", json=payload,
                              headers={"Authorization": f"Bearer {api_key}"}, timeout=90)
        if response.status_code != 429:
            break
        if attempt == 4:
            break
        delay = _groq_retry_delay(response, attempt)
        print(f"Groq rate limit reached; retrying in {delay:.1f}s (attempt {attempt + 1}/4).")
        time.sleep(delay)
    if not response.is_success:
        try:
            detail = response.json().get("error", response.text)
        except ValueError:
            detail = response.text
        raise RuntimeError(f"Groq API returned HTTP {response.status_code}: {detail}")
    data = response.json()
    message = data["choices"][0]["message"]
    output = json.loads(message["content"])
    action = output.pop("action")
    decision = {"name": action, "id": f"groq_{uuid.uuid4().hex[:8]}", "input": output}
    assistant_content = [{"type": "text", "text": message["content"]}]
    usage = data.get("usage", {})
    return decision, assistant_content, usage.get("prompt_tokens", 0), usage.get("completion_tokens", 0)


def _groq_retry_delay(response, attempt: int) -> float:
    """Honor Groq's retry hint while keeping retries bounded."""
    delay = None
    retry_after = response.headers.get("retry-after")
    if retry_after:
        try:
            delay = float(retry_after)
        except ValueError:
            pass
    if delay is None:
        try:
            message = response.json().get("error", {}).get("message", "")
            match = re.search(r"try again in ([0-9.]+)s", message, re.IGNORECASE)
            if match:
                delay = float(match.group(1))
        except (ValueError, AttributeError):
            pass
    if delay is None:
        delay = min(2 ** attempt, 30)
    return min(max(delay + 0.2, 0.5), 60.0)


def run_agent(command: str, base_url: str | None = None, dummy: bool = False,
              max_steps: int = 60, provider: str = "claude", seed: int | None = None) -> dict:
    """Run the visual Fetch loop using Claude, Groq, or plumbing-only dummy mode."""
    max_steps = min(max(1, max_steps), 60)
    if dummy:
        provider = "dummy"
    if provider not in ("claude", "groq", "dummy"):
        raise ValueError("provider must be claude, groq, or dummy")
    model = os.getenv("GROQ_MODEL", "qwen/qwen3.8-27b") if provider == "groq" else os.getenv("CLAUDE_MODEL", "claude-sonnet-5-5")
    if not dummy:
        if provider == "groq":
            api_key = os.getenv("GROQ_API_KEY")
            if not api_key:
                raise RuntimeError("Set GROQ_API_KEY or use --dummy")
            claude = None
        else:
            import anthropic
            if not os.getenv("ANTHROPIC_API_KEY"):
                raise RuntimeError("Set ANTHROPIC_API_KEY or use --dummy")
            claude = anthropic.Anthropic()
    else:
        claude = None

    client = EngineClient(base_url=base_url or os.getenv("ENGINE_URL", "http://127.0.0.1:8080"))
    run_id = f"run_{int(time.time())}_{uuid.uuid4().hex[:6]}"
    logger = FetchLogger(run_id, command)
    state = client.state()
    image, _ = client.look()
    messages: list[dict] = [{"role": "user", "content": [
        {"type": "text", "text": _status_text(command, state, None, 0)}, _image_block(image)]}]
    last_result = None
    verified_delivery = False
    history: list[dict] = []
    recovery_turn_pending = False
    for _ in range(max_steps):
        if state.get("state") == "DONE":
            break
        if logger.step == 25:
            print("Soft limit reached (25 actions); continuing toward the 60-action hard limit.")
        if state.get("state") == "BLOCKED":
            # Engine has stopped motion and recorded a retry. Recover with one
            # bounded reverse action, then a small turn before asking the model
            # to choose the next task action.
            decision = {"name": "move", "id": f"recovery_{logger.step}",
                        "input": {"distance_m": -0.15,
                                  "what_i_see": "Collision reported by robot status.",
                                  "why": "Back away from the contact before resuming."}}
            recovery_turn_pending = True
            assistant_content = [{"type": "tool_use", "id": decision["id"],
                                  "name": decision["name"], "input": decision["input"]}]
            tokens_in = tokens_out = 0
        elif recovery_turn_pending:
            decision = {"name": "turn", "id": f"recovery_{logger.step}",
                        "input": {"degrees": 30,
                                  "what_i_see": "The robot backed away from a collision.",
                                  "why": "Change heading slightly to clear the obstacle."}}
            recovery_turn_pending = False
            assistant_content = [{"type": "tool_use", "id": decision["id"],
                                  "name": decision["name"], "input": decision["input"]}]
            tokens_in = tokens_out = 0
        elif dummy:
            # Plumbing mode exercises a single safe action without inventing scene facts.
            decision = {"name": "turn", "id": f"dummy_{logger.step}",
                        "input": {"degrees": 25, "what_i_see": "The view needs a wider scan.",
                                  "why": "Rotate to inspect another part of the room."}}
            assistant_content = [{"type": "tool_use", "id": decision["id"],
                                 "name": decision["name"], "input": decision["input"]}]
            tokens_in = tokens_out = 0
        elif provider == "groq":
            decision, assistant_content, tokens_in, tokens_out = _groq_decision(
                messages, model, api_key)
        else:
            response = claude.messages.create(
                model=model, max_tokens=512, system=FETCH_SYSTEM_PROMPT,
                tools=FETCH_TOOLS, tool_choice={"type": "any", "disable_parallel_tool_use": True},
                messages=messages)
            calls = [block for block in response.content if block.type == "tool_use"]
            if len(calls) != 1:
                raise RuntimeError("Claude must return exactly one tool call per decision")
            decision = {"name": calls[0].name, "id": calls[0].id, "input": calls[0].input}
            assistant_content = response.content
            tokens_in = response.usage.input_tokens
            tokens_out = response.usage.output_tokens
        tool, args = decision["name"], decision["input"]
        if tool not in ACTION_PATH:
            raise RuntimeError(f"Unsupported action: {tool}")
        if tool == "move":
            result = client.move(args["distance_m"])
        elif tool == "turn":
            result = client.turn(args["degrees"])
        elif tool == "pickup":
            result = client.pickup()
        else:
            result = client.drop()
        previous_state = state
        state = client.state()
        logger.record(tool, args, result, image, previous_state, provider if dummy else model,
                      tokens_in, tokens_out)
        last_result = result
        history.append({"action": tool, "result": result,
                        "observation": args.get("what_i_see"), "state": state.get("state"),
                        "tokens_in": tokens_in, "tokens_out": tokens_out})
        if tool == "drop":
            verified_delivery = bool(result.get("in_zone", False))
        if tool in ("move", "turn"):
            image, _ = client.look()
            next_user_content = [{"type": "text", "text": _status_text(command, state, last_result,
                                                                           logger.step, history)}]
            if provider != "groq":
                next_user_content.insert(0, {"type": "tool_result", "tool_use_id": decision["id"],
                                             "content": str(result)})
            next_user_content.append(_image_block(image))
        else:
            next_user_content = [{"type": "text", "text": _status_text(command, state, last_result,
                                                                           logger.step, history)}]
            if provider != "groq":
                next_user_content.insert(0, {"type": "tool_result", "tool_use_id": decision["id"],
                                             "content": str(result)})
        if not dummy:
            messages.extend([{"role": "assistant", "content": assistant_content},
                             {"role": "user", "content": next_user_content}])
            _keep_latest_image(messages)
        if state.get("state") == "DONE":
            break
    complete = (verified_delivery and state.get("state") == "DONE" and
                state.get("outcome") == "pass")
    outcome = state.get("outcome") if state.get("state") == "DONE" else "incomplete"
    outcome = outcome or ("pass" if complete else "fail_not_reached")
    logger.finalize(seed=seed, outcome=outcome, total_tokens_in=sum(
        item.get("tokens_in", 0) for item in history), total_tokens_out=sum(
        item.get("tokens_out", 0) for item in history))
    return {"outcome": outcome, "state": state.get("state"),
            "retry_count": state.get("retry_count"), "steps": logger.step,
            "run_id": run_id, "last_action_result": last_result,
            "history": history}
