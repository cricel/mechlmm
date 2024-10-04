
class Item(BaseModel):
    '''detail break down of item'''

    name: str = Field(..., description="the name of the object detected")
    position: List[float] = Field(..., description="the bounding box coordinate of the object detected, such as ['top_left_x', 'top_left_y', 'bottom_right_x', 'bottom_right_y']")
    features: List[str] = Field(..., description="the key features of the object detected")

class ItemList(BaseModel):
    '''a list of items description'''

    items: List[Item] = Field(..., description="the list of items")
    description: str = Field(..., description="overall description of what is seen in the image")
