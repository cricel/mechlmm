from ollama import Client
client = Client(host='http://131.123.41.132:11434')

print ("------ Text LLM ------")
res = client.chat(model='llama3', messages=[
  {
    'role': 'user',
    'content': 'Hi',
  },
])
print(res['message']['content'])

print ("------ Image LLM ------")
res = client.chat(
	model="llava:34b",
	messages=[
		{
			'role': 'user',
			'content': 'give me a one sentence description of what happen in this image:',
			'images': ['./messy_room.jpg']
		}
	]
)

print(res['message']['content'])