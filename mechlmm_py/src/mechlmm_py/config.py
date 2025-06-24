# src/mypackage/config.py
from pydantic_settings import BaseSettings, SettingsConfigDict

class Settings(BaseSettings):
    mechlmm_ip:  str = "127.0.0.1"
    endpoint: str = "https://api.example.com"
    timeout:  float = 5.0

    model_config = SettingsConfigDict(
        env_file=".env",       # read local file if present
        extra="ignore",        # silently drop unknown keys
    )

settings = Settings()          # load once at import time
