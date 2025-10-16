import time
from multiprocessing import Process, Queue, Pipe

from DummyDetector import DummyDetectorProcess as DetectorProcess
from Decision import DecisionProcess
from Tracker import TrackerProcess
from DummyDelta import Delta

import sys
sys.path.append(".")

main_in, main_out = Pipe()
detector_in, detector_out = Pipe()
tracker_in, tracker_out = Pipe()
decision_in, decision_out = Pipe()
delta_in, delta_out = Pipe()

detector_p = Process(target=DetectorProcess.run, args=(main_out, detector_in))
tracker_p = Process(target=TrackerProcess.run, args=(detector_out, tracker_in))
decision_p = Process(target=DecisionProcess.run, args=(tracker_out, decision_in))
delta_p = Process(target=Delta.delta_process, args=(decision_out, delta_in))

processes = [
    detector_p,
    tracker_p,
    decision_p,
    delta_p,
]

for p in processes:
    p.daemon = True  # Ensure processes are killed when the main process exits
    p.start()


try:
    while True:
        if delta_out.poll(timeout=5):
            msg = delta_out.recv()
            print(msg)

except KeyboardInterrupt:
    main_in.send(("EXIT", None))

    for p in processes:
        p.join(timeout=5)
        p.terminate()
        print(f"Process {p.name} terminated.")