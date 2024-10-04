from ollama import Client
client = Client(host='http://131.123.41.132:11434')

print ("------ Text LLM ------")
res = client.chat(model='llava:34b', messages=[
  {
    'role': 'user',
    'content': 'Hi',
  },
])
print(res['message']['content'])

print ("------ Image LLM ------")
res = client.chat(
	model="llava",
	messages=[
		{
			'role': 'user',
			'content': 'what is the pixel position of the trash can in the image, give me the bounding box of the object',
			'images': ['./test.jpg']
		}
	]
)

print(res['message']['content'])