#!/usr/bin/env python3
"""Validate fixed motor inventory and generate RT configuration, no dependencies."""
import argparse, hashlib, json, math
from pathlib import Path
DM={'DM4310':0,'DM4310_48V':1,'DM4340':2,'DM4340_48V':3,'DM6006':4,
    'DM8006':5,'DM8009':6,'DM-J8009-2EC':6,'DM-J8009P-2EC':6,'DM10010L':7,'DM10010':8,
    'DMH3510':9,'DMH6215':10,'DMG6220':11,'DM2325':12}
DJI={'GM6020':1,'M3508':2,'M2006':3}
def integer(v,lo,hi,name):
    if type(v) is not int or not lo<=v<=hi: raise ValueError(f'{name}: expected integer {lo}..{hi}')
    return v
def number(v,lo,hi,name,positive=False):
    if type(v) not in (int,float) or not math.isfinite(v) or not lo<=v<=hi or (positive and v<=0):
        raise ValueError(f'{name}: invalid finite range')
    return float(v)
def validate(data):
    if data.get('schema_version')!=1 or data.get('rt_tick_ms')!=1:raise ValueError('schema_version/rt_tick_ms must be 1')
    for key in ['status_hz','feedback_hz']:
        hz=integer(data[key],1,1000,key)
        if 1000%hz:raise ValueError(key+': must divide 1000 Hz')
    if data['status_hz']>data['feedback_hz']:raise ValueError('status_hz exceeds feedback_hz')
    bitrate=integer(data['can_bitrate'],100000,1000000,'can_bitrate')
    # This board driver is fixed to 1 Mbps; never validate a fictitious budget.
    if bitrate!=1000000:raise ValueError('board CAN driver currently requires 1000000 bps')
    fraction=number(data['can_budget_fraction'],.1,.85,'can_budget_fraction')
    motors=data['motors'];integer(len(motors),1,16,'motor count')
    ids=set();receivers={};groups={};load={1:0,2:0};rows=[];counts={'dm':0,'dji':0}
    for m in motors:
        n=integer(m['number'],1,65535,'number')
        if n in ids:raise ValueError('duplicate motor number')
        ids.add(n);family=m['family'];model=m['model'];bus=integer(m['bus'],1,2,'bus')
        if family not in counts:raise ValueError('unknown family')
        counts[family]+=1;cid=integer(m['can_id'],1,2047,'can_id')
        offline=integer(m['offline_ms'],2,1000,'offline_ms')
        publish=integer(m.get('feedback_publish_hz',500 if family=='dm' else 1000),1,data['feedback_hz'],'feedback_publish_hz')
        if 1000%publish:raise ValueError('feedback_publish_hz must divide 1000')
        mit=integer(m['mit_period_ms'],1,1000,'mit_period_ms')
        native=integer(m['native_period_ms'],1,1000,'native_period_ms')
        fb=integer(m['expected_feedback_hz'],1,1000,'expected_feedback_hz')
        if offline<2000/fb:raise ValueError('offline_ms must allow at least two feedback intervals')
        limits=m['control_limits'];speed=number(limits['velocity_rad_s'],0,1000,'velocity limit',True)
        torque=number(limits.get('torque_nm',0),0,1000,'torque limit')
        pid=m.get('pid',{});skp=number(pid.get('speed_kp',0),0,1e6,'speed_kp');ski=number(pid.get('speed_ki',0),0,1e6,'speed_ki');skd=number(pid.get('speed_kd',0),0,1e6,'speed_kd')
        so=number(pid.get('speed_output',0),0,30000,'speed_output');pkp=number(pid.get('position_kp',0),0,1e6,'position_kp');po=number(pid.get('position_output',0),0,1e6,'position_output')
        if family=='dm':
            if model not in DM:raise ValueError('unknown DM model')
            drv=DM[model];fid=integer(m['feedback_id'],0,2047,'feedback_id')
            if cid+0x200>0x7ff:raise ValueError('DM native mode CAN ID overflows standard ID')
            key=(bus,fid,cid&15)
            if key in receivers:raise ValueError('ambiguous shared DM feedback ID/low nibble')
            # Reject mixed DJI/DM feedback routes (fanout cannot classify reliably).
            if (bus,fid,'dji') in receivers:raise ValueError('DM/DJI feedback ID collision')
            receivers[key]=1;receivers[(bus,fid,'dm')]=1
            p=m['protocol_limits'];pmax=number(p['pmax'],0,10000,'pmax',True);vmax=number(p['vmax'],0,10000,'vmax',True);tmax=number(p['tmax'],0,10000,'tmax',True)
            if speed>vmax or torque>tmax:raise ValueError('application limit exceeds mapping')
            if fb<1000/min(mit,native):raise ValueError('expected DM feedback must cover control-response rate')
            load[bus]+=135*(1000/min(mit,native)+fb)
            fam=1
        else:
            if model not in DJI:raise ValueError('unknown DJI model')
            drv=DJI[model];integer(cid,1,7 if model=='GM6020' else 8,'DJI can_id')
            fid=(0x204 if model=='GM6020' else 0x200)+cid
            if m.get('feedback_id',fid)!=fid:raise ValueError('DJI feedback ID does not match hardware ID')
            if (bus,fid,'dji') in receivers or (bus,fid,'dm') in receivers:raise ValueError('DJI feedback ID collision')
            receivers[(bus,fid,'dji')]=1
            group=(0x1ff if cid<=4 else 0x2ff) if model=='GM6020' else (0x200 if cid<=4 else 0x1ff)
            key=(bus,group)
            if key in groups and groups[key]!=mit:raise ValueError('DJI shared group periods must agree')
            groups[key]=mit;load[bus]+=135*fb
            pmax=1e6;vmax=speed;tmax=0;fam=2
        rows.append([n,fam,(fam<<16)|drv,drv,bus,cid,fid,offline,1000//publish,mit,native,pmax,vmax,tmax,speed,torque,skp,ski,skd,so,pkp,po])
    if counts['dm']>4 or counts['dji']>12:raise ValueError('driver instance capacity exceeded')
    for (bus,_),period in groups.items():load[bus]+=135*1000/period
    for bus,bps in load.items():
        if bps>bitrate*fraction:raise ValueError(f'CAN{bus}: estimated {bps:.0f} bps exceeds configured {fraction:.0%} budget')
    # Native DM control IDs and all management IDs must not collide with DJI group commands.
    dm_tx=set();dji_tx=set(groups)
    for row in rows:
        if row[1]==1:
            for offset in (0,0x100,0x200):
                key=(row[4],row[5]+offset)
                if key in dm_tx or key in dji_tx or key[1]==0x7ff:raise ValueError('control CAN ID collision')
                dm_tx.add(key)
    return rows,load

def render(data,raw):
    rows,load=validate(data)
    def c(v):return (format(v,'.9g')+('f' if '.' in format(v,'.9g') or 'e' in format(v,'.9g') else '.0f')) if isinstance(v,float) else str(v)+'U'
    digest=int.from_bytes(hashlib.sha256(raw).digest()[:4],'little')
    text='/* Generated from config/motors.json; do not edit. */\n#include "amp_motor_config.h"\n'
    text+='const struct amp_motor_config amp_motor_config[]={\n'+''.join('    {'+', '.join(c(v) for v in row)+'},\n' for row in rows)+'};\n'
    text+=f'const uint32_t amp_motor_config_count={len(rows)}U, amp_motor_config_hash=0x{digest:08x}U;\n'
    text+=f'const uint32_t amp_motor_status_hz={data["status_hz"]}U, amp_motor_feedback_hz={data["feedback_hz"]}U;\n'
    return text,load

def main():
    project=Path(__file__).resolve().parents[1];p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=project/'config/motors.json');p.add_argument('--out',type=Path,default=project/'application/motor_config.generated.h');a=p.parse_args()
    raw=a.config.read_bytes();text,load=render(json.loads(raw),raw)
    if not a.out.exists() or a.out.read_text()!=text:a.out.write_text(text)
    print('Motor config valid; conservative CAN budget:',', '.join(f'CAN{b}={v:.0f} bps' for b,v in load.items()))
if __name__=='__main__':main()
