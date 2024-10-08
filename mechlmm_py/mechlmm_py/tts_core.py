from elevenlabs import play
from elevenlabs.client import ElevenLabs

client = ElevenLabs(
  api_key="sk_37b0037ad75e885f53948b5dce19a249b032984e2a9e9190", # Defaults to ELEVEN_API_KEY
)

audio = client.generate(
  text="Hello! 你好!",
  voice="Rachel",
  model="eleven_multilingual_v2"
)
play(audio)