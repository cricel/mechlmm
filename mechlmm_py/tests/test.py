from mechlmm_py.config import settings
from mechlmm_py.debug_core import DebugCore
from mechlmm_py.db_core import DB_Core
import mechlmm_py.utilities_core as utilities_core

print("Test Debug Core ==============")
debug_core = DebugCore()
debug_core.verbose = 3
debug_core.log_error("error")
debug_core.log_info("info")
debug_core.log_warning("warning")
debug_core.log_print("print")

print("Test DB Core ==============")
db_core = DB_Core()

print("Test Utilities Core ==============")
# utilities_core.frame_to_jpg(utilities_core.query_video_frame(7), "test.jpg")

