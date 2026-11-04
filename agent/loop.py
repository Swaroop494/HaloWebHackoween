import os
import json
import base64
from agent.engine_client import EngineClient
from agent import prompts, tools
from agent.logger import RunLogger

class DummyAgent:
    def decide(self, state, depth_info, image_bytes):
        # Simple dummy: if not holding and see something? just turn or move
        if state.get('holding'):
            return {'tool': 'drop', 'args': {'what_i_see': 'carrying object', 'why': 'attempt to deliver'}}
        # try to move forward a bit
        return {'tool': 'move', 'args': {'distance_m': 0.3, 'what_i_see': 'room ahead', 'why': 'explore'}}

def run_once(seed, command, dummy=True):
    client = EngineClient()
    run_id = f'run_{seed:03d}'
    client.run_start(seed, run_id, command)
    logger = RunLogger(run_id, seed, command)
    agent = DummyAgent()
    for step in range(60):
        st = client.state()
        frame, headers = client.look()
        dep = client.depth()
        dec = agent.decide(st, dep, frame)
        tool = dec['tool']
        args = dec['args']
        res = None
        if tool == 'move':
            res = client.move(args['distance_m'])
        elif tool == 'turn':
            res = client.turn(args['degrees'])
        elif tool == 'pickup':
            res = client.pickup()
        elif tool == 'drop':
            res = client.drop()
        logger.log_step(step, tool, args, res, frame, st.get('state'), st.get('collided', False))
        if st.get('holding') is False and res and res.get('state') == 'Done':
            break
        # end if done
        st2 = client.state()
        if st2.get('state') == 'Done':
            break
    end = client.run_end()
    logger.finish(end.get('outcome'), step+1)
    return end
