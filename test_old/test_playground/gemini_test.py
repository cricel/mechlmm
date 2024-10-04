import google.generativeai as genai
genai.configure(api_key="AIzaSyBd5bfYtr6mrJLjhiMp_VU7vmijqwUX_H8")
model = genai.GenerativeModel(model_name="gemini-1.5-pro-latest")
import PIL.Image
goats = PIL.Image.open("test.jpg")
prompt = 'Return bounding boxes around the trash can, for each one return [ymin, xmin, ymax, xmax]'
response = model.generate_content([goats, prompt])
print(response.text)