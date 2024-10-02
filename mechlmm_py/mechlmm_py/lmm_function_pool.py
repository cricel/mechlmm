from pydantic import BaseModel, Field

class navigation(BaseModel):
    """Navigate to the target location"""

    target_name: str = Field(..., description="the name of the target location")

class manipulation(BaseModel):
    """Manipulate the target object"""

    target_name: str = Field(..., description="the name of the target object")
