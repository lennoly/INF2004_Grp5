from pathlib import Path
import subprocess,os,hashlib,struct,json
root=Path.cwd();out=root/'tmp/final-review'
env=os.environ.copy();env['MAKEFLAGS']='-B'
r=subprocess.run(['sh','tools/build_buddy3.sh'],env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
(out/'buddy3-full-build.log').write_text(r.stdout)
print('Full rebuild exit:',r.returncode,flush=True)
for line in r.stdout.splitlines():
 if any(x in line.lower() for x in ['warning:','error:','text\t','69321','finished building target:','elf2uf2']): print(line,flush=True)
if r.returncode: print(r.stdout[-5000:]);raise SystemExit(r.returncode)
cache=Path.home()/'.cache/picocar-buddy3'
mismatches=[]
for p in (root/'app_program').rglob('*'):
 if p.is_file() and p.suffix in {'.c','.h'}:
  other=cache/'template/app_program'/p.relative_to(root/'app_program')
  if not other.exists() or p.read_bytes()!=other.read_bytes(): mismatches.append(str(p))
assert not mismatches,mismatches
extra=[str(p) for p in (cache/'template/app_program').rglob('*') if p.is_file() and p.suffix in {'.c','.h'} and not (root/'app_program'/p.relative_to(cache/'template/app_program')).exists()]
assert not extra,extra
uf=root/'firmware/picocar_TEST_BUDDY3_uart.uf2';data=uf.read_bytes();assert len(data)%512==0
assert data==(cache/'template/build_make/mtk3pico_smp0_uart.uf2').read_bytes()
n=len(data)//512;payload=bytearray()
for i in range(n):
 b=data[i*512:(i+1)*512];m0,m1,flags,address,size,no,total,family=struct.unpack_from('<8I',b)
 assert (m0,m1)==(0x0A324655,0x9E5D5157) and struct.unpack_from('<I',b,508)[0]==0x0AB16F30
 assert no==i and total==n and size==256 and flags&0x2000 and family==0xe48bff56
 assert address==0x10000000+i*256
 payload.extend(b[32:32+size])
sp,reset=struct.unpack_from('<2I',payload,256)
assert 0x20000000<=sp<=0x20042000 and reset&1 and 0x10000000<=reset<0x10200000,(hex(sp),hex(reset))
for marker in [b'Buddy 3:',b'Configure map=',b'barcode mapping before start',b'executing %s']:
 assert marker in payload,marker
info={'size':len(data),'blocks':n,'sha256':hashlib.sha256(data).hexdigest(),'stack_pointer':hex(sp),'reset_vector':hex(reset),'source_cache_matches':True,'family':'RP2040','full_build':True}
(out/'uf2-verification.json').write_text(json.dumps(info,indent=2));print(json.dumps(info,indent=2),flush=True)
