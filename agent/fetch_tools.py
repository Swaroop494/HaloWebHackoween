"""Claude tool schema and engine action mapping for Fetch."""
FETCH_TOOLS = [
    {"name": "move", "description": "Move forward or backward by a distance in metres.",
     "input_schema": {"type": "object", "properties": {
         "distance_m": {"type": "number", "minimum": -2, "maximum": 2,
                         "description": "Required signed distance in metres, from -2 to 2."},
         "what_i_see": {"type": "string", "description": "Required brief visual observation."},
         "why": {"type": "string", "description": "Required reason for this action."}},
         "additionalProperties": False,
         "required": ["distance_m", "what_i_see", "why"]}},
    {"name": "turn", "description": "Turn in place in degrees; positive turns right.",
     "input_schema": {"type": "object", "properties": {
         "degrees": {"type": "number", "minimum": -180, "maximum": 180,
                      "description": "Required signed turn angle in degrees. Positive turns right."},
         "what_i_see": {"type": "string", "description": "Required brief visual observation."},
         "why": {"type": "string", "description": "Required reason for this action."}},
         "additionalProperties": False,
         "required": ["degrees", "what_i_see", "why"]}},
    {"name": "pickup", "description": "Attempt to pick up the item directly in front of the robot.",
     "input_schema": {"type": "object", "properties": {
         "what_i_see": {"type": "string", "description": "Required brief visual observation."},
         "why": {"type": "string", "description": "Required reason for this action."}},
         "additionalProperties": False,
         "required": ["what_i_see", "why"]}},
    {"name": "drop", "description": "Release the held item on the marked area.",
     "input_schema": {"type": "object", "properties": {
         "what_i_see": {"type": "string", "description": "Required brief visual observation."},
         "why": {"type": "string", "description": "Required reason for this action."}},
         "additionalProperties": False,
         "required": ["what_i_see", "why"]}},
]

ACTION_PATH = {"move": "/move", "turn": "/turn", "pickup": "/pickup", "drop": "/drop"}
# Observation-only calls are performed by the loop between decisions, never by
# a model tool call. This guarantees a fresh image while keeping one action per
# decision.
