import ollama

res = ollama.chat(
	model="llama3.2",
	messages=[
		{
			'role': 'user',
			'content': 'Describe this image:',
			'images': ['./art.jpg']
		}
	]
)

print(res['message']['content'])