import time
from datetime import datetime


tt = int(time.time())
print(tt)


readable_time = datetime.fromtimestamp(tt)

print("Current date and time:", readable_time)