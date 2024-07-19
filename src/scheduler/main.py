import sys
from multiprocessing import Process, Event
from scheduler import ToDoScheduler

def run_scheduler(stop_event):
    scheduler = ToDoScheduler(stop_event)
    scheduler.schedule_task()

if __name__ == "__main__":
    stop_event = Event()
    process = Process(target=run_scheduler, args=(stop_event,))
    
    if len(sys.argv) > 1 and sys.argv[1] == "Stop":
        print("Stopping the scheduler.")
        stop_event.set()
        process.terminate()
        process.join()
        print("Scheduler has been stopped.")
    else:
        process.start()
        print("Scheduler has been started in the background.")
        try:
            process.join()
        except KeyboardInterrupt:
            print("Stopping the scheduler.")
            stop_event.set()
            process.terminate()
            process.join()
            print("Scheduler has been stopped.")
