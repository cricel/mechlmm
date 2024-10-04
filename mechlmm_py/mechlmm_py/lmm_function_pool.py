from pydantic import BaseModel, Field

class navigation(BaseModel):
    """Navigate to the target location"""

    target_name: str = Field(..., description="the name of the target location")

class manipulation(BaseModel):
    """Manipulate the target object"""

    target_name: str = Field(..., description="the name of the target object")

class structure_json_output(BaseModel):
    """format the question to json format base on the arg provided"""