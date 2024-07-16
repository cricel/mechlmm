import google.generativeai as genai
import PIL.Image

GOOGLE_API_KEY="AIzaSyCSjCUGUydxiSsWmlMLEU_nfqtu70HPGZo"
genai.configure(api_key=GOOGLE_API_KEY)

# model = genai.GenerativeModel('gemini-1.5-flash')

# # Chat
# # response = model.generate_content("my dog`s name is Max")

# # Image
# img = PIL.Image.open('./test_img.png')
# # # response = model.generate_content(img)
# # response = model.generate_content(["Tell me what is in the picture, and give me their bounding box coordinate.", img], stream=True)
# # response.resolve()

# # print(response.text)

# # Chat History
# chat = model.start_chat(history=[])
# response = chat.send_message("My name is Shawn")
# print(response.text)
# print("=====")
# # response = chat.send_message(["Tell me what is in the picture, and give me their bounding box coordinate.", img], stream=True)
# # response.resolve()
# # print(response.text)
# # print("=====")
# print(chat.history)
# print("=====")
# response = chat.send_message("what is my name?")
# print(response.text)
# print("=====")
# response = chat.send_message("what is my dog name?")
# print(response.text)
# print("=====")
# response = chat.send_message("what is my name?")
# print(response.text)
# print("=====")


# # Function Calling
# def set_light_values(brightness, color_temp):
#     """Set the brightness and color temperature of a room light. (mock API).

#     Args:
#         brightness: Light level from 0 to 100. Zero is off and 100 is full brightness
#         color_temp: Color temperature of the light fixture, which can be `daylight`, `cool` or `warm`.

#     Returns:
#         A dictionary containing the set brightness and color temperature.
#     """
#     print("0-0-0-0-0")
#     print(brightness)
#     return {
#         "brightness": brightness,
#         "colorTemperature": color_temp
#     }

# def dim_lights(brightness: float) -> bool:
#     """Dim the lights.

#     Args:
#       brightness: The brightness of the lights, 0.0 is off, 1.0 is full.
#     """
#     print(f"Lights are now set to {brightness:.0%}")
#     return True

# model = genai.GenerativeModel(model_name='gemini-1.5-flash',
#                               tools=[dim_lights])

# chat = model.start_chat()
# response = chat.send_message('turn the light off.')
# # print(response.text)


def power_disco_ball(power: bool) -> bool:
    """Powers the spinning disco ball."""
    print(f"Disco ball is {'spinning!' if power else 'stopped.'}")
    return True


def start_music(energetic: bool, loud: bool, bpm: int) -> str:
    """Play some music matching the specified parameters.

    Args:
      energetic: Whether the music is energetic or not.
      loud: Whether the music is loud or not.
      bpm: The beats per minute of the music.

    Returns: The name of the song being played.
    """
    print(f"Starting music! {energetic=} {loud=}, {bpm=}")
    return "Never gonna give you up."


def dim_lights(brightness: float) -> bool:
    """Dim the lights.

    Args:
      brightness: The brightness of the lights, 0.0 is off, 1.0 is full.
    """
    print(f"Lights are now set to {brightness:.0%}")
    return True

function_handler = {
    "power_disco_ball": power_disco_ball,
    "start_music": start_music,
    "dim_lights": dim_lights,
}

# Set the model up with tools.
house_fns = [power_disco_ball, start_music, dim_lights]

model = genai.GenerativeModel(model_name="gemini-1.5-flash", tools=house_fns)

# Call the API.
chat = model.start_chat()
# response = chat.send_message("Turn this place into a party!")
# response = chat.send_message("Turn this place into a party!")
response = chat.send_message("tell me a joke")

# print(response)

function_responses = {}
trigger_function_call = False

# Print out each of the function calls requested from this single call.
for part in response.parts:
    if fn := part.function_call:
        trigger_function_call = True
        args = ", ".join(f"{key}={val}" for key, val in fn.args.items())
        print(f"{fn.name}({args})")
        function_responses[fn.name] = function_handler[fn.name](**fn.args)

if(trigger_function_call):
    # Build the response parts.
    response_parts = [
        genai.protos.Part(function_response=genai.protos.FunctionResponse(name=fn, response={"result": val}))
        for fn, val in function_responses.items()
    ]

    response = chat.send_message(response_parts)
    print(response.text)
else:
    print(response.text)
