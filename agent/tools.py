TOOLS = [
    {
        "name": "move",
        "description": "Move the robot forward or backward by distance_m metres.",
        "input_schema": {
            "type": "object",
            "properties": {
                "distance_m": {"type": "number", "minimum": -2.0, "maximum": 2.0},
                "what_i_see": {"type": "string"},
                "why": {"type": "string"},
            },
            "required": ["distance_m", "what_i_see", "why"]
        }
    },
    {
        "name": "turn",
        "description": "Turn the robot in place by degrees. Positive is to the right.",
        "input_schema": {
            "type": "object",
            "properties": {
                "degrees": {"type": "number", "minimum": -180.0, "maximum": 180.0},
                "what_i_see": {"type": "string"},
                "why": {"type": "string"},
            },
            "required": ["degrees", "what_i_see", "why"]
        }
    },
    {
        "name": "pickup",
        "description": "Attempt to pick up the object within reach.",
        "input_schema": {
            "type": "object",
            "properties": {
                "what_i_see": {"type": "string"},
                "why": {"type": "string"},
            },
            "required": ["what_i_see", "why"]
        }
    },
    {
        "name": "drop",
        "description": "Release the held object.",
        "input_schema": {
            "type": "object",
            "properties": {
                "what_i_see": {"type": "string"},
                "why": {"type": "string"},
            },
            "required": ["what_i_see", "why"]
        }
    },
]
