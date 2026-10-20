"""Numerical model regression against original PC kernel (not C execution).
Run: python 数值回归验证.py PATH_TO_ORIGINAL_pid_nei_he.py
"""
import math, random, sys, types
from pathlib import Path

def step(s, target, actual, dt, kp=10.0, ki=0.05, kd=20.0):
    # Mirrors the portable C update in original C operation order.
    if dt<0.05: dt=0.5
    if dt>1: dt=1.0
    e=target-actual
    ep=0.0 if abs(e)<0.3 else e
    if not ((s['out']>=100 and e>0) or (s['out']<=0 and e<0)):
        s['integral']+=e*dt
    s['integral']=max(0.0,min(100.0/ki,s['integral']))
    d=0.0 if s['last'] is None else -(actual-s['last'])/dt
    if abs(d)<0.1/dt: d=0.0
    s['d']+=0.2*(d-s['d'])
    raw=max(0.0,min(100.0,kp*ep+ki*s['integral']+kd*s['d']))
    s['out']=max(s['out']-5*dt,min(s['out']+5*dt,raw))
    s['last']=actual
    return s['out']

def state(): return dict(integral=0.0,out=0.0,d=0.0,last=None)

def main():
    path=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parents[1]/'原版参考/原版电脑端参考.py' if len(sys.argv)>1 else Path(__file__).resolve().parents[1]/'原版参考/原版电脑端参考.py'
    # Load only the actual original WenDuPID section: no third-party autotune.
    code=path.read_text(encoding='utf-8-sig').split('#  继电自整定适配层')[0]
    ns={}
    exec(compile(code,str(path),'exec'),ns)
    cls=ns['WenDuPID']
    rng=random.Random(20261019)
    worst=0.0
    total=0
    for base_dt in (0.35,0.5,0.6,1.0):
        p=cls(10,0.05,20,kong_zhi_zhou_qi_s=base_dt,si_qu=0.3,
              lv_bo_xi_shu=0.2,d_si_qu=1,mei_miao_zui_da_bian_hua=5,
              fen_bian_lv=0)
        s=state()
        for i in range(2400):
            actual=25+75*(1-math.exp(-i/220))+0.25*math.sin(i/45)+rng.uniform(-0.04,0.04)
            target=100 if i<1300 else (80 if i<1800 else 105)
            a=p.ji_suan(actual,target,dt=base_dt)
            b=step(s,target,actual,base_dt)
            diff=max(abs(a-b),abs(p.ji_fen-0.05*s['integral']),abs(p.wei_fen_lv_bo-s['d']))
            worst=max(worst,diff)
            assert diff<1e-9,(i,base_dt,diff)
            total+=1
    # Independent behavior checks: ramp, P-only deadband with active I,
    # no setpoint derivative kick, normalized D threshold, dt fallback/cap,
    # isolated instances and reset.
    s=state(); assert step(s,100,25,0.5)==2.5
    assert step(s,100,25,0.5)==5.0
    s=state(); step(s,100,99.8,0.5); assert 0<s['out']<0.01
    s=state(); step(s,100,90,0.5); step(s,110,90,0.5); assert s['d']==0
    s=state(); step(s,100,90,0.5); step(s,100,90.05,0.5); assert s['d']==0
    s=state(); step(s,100,90,0.5); step(s,100,90.2,0.5); assert s['d']<0
    a=state(); b=state(); assert step(a,100,25,0)==step(b,100,25,0.5)
    a=state(); b=state(); assert step(a,100,25,5)==step(b,100,25,1)
    a=state(); b=state(); step(a,100,25,0.5); assert b==state()
    a=state(); assert a['out']==0 and a['last'] is None
    print('PASS: 9600 model/原版参考 steps; edge cases passed; max error %.3g'%worst)
    print('This is Python double-precision model regression, not C51 runtime validation.')

if __name__=='__main__': main()
