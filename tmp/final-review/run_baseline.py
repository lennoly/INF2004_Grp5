from pathlib import Path
import subprocess
out=Path("tmp/final-review")
for name,cmd in [("buddy3-baseline",["python3","testing/subsystem3/run_host_tests.py"]),("general-baseline",["make","-B","-C","tests/host"])]:
 r=subprocess.run(cmd,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
 (out/(name+".log")).write_text(r.stdout)
 print(name, "exit",r.returncode,flush=True)
 print(r.stdout[-2200:],flush=True)
 if r.returncode: raise SystemExit(r.returncode)
