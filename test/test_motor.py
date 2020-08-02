import os
import sys
import getpass

env=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar/driver/actor'))
sys.path.insert(0, env)

from jga25_370 import JGA25_370

if __name__ == "__main__":
	motorjga25_370 = JGA25_370(18, 6, 13)

	print("w/s: accelerate")
	print("q:   stops the engines")
	print("x:   exit program")

	motorjga25_370.test_engine()
