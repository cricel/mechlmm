import json

list = ["""function_call {
  name: "dim_lights"
  args {
    fields {
      key: "brightness"
      value {
        number_value: 0
      }
    }
  }
}"""
]

print(json.dumps(list, indent=2))