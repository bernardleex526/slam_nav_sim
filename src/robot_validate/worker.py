import json
import sys
from .runner import worker

if __name__ == "__main__":
    sys.exit(worker(json.loads(open(sys.argv[1]).read()), sys.argv[2]))
