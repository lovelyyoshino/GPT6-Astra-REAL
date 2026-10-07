import json
from pathlib import Path
from right_pick.recording import Recorder
from right_pick.fast_ros import ROSRightArm
from right_pick.fast_codex import CodexDecisionClient
from right_pick.fast_observation import SubprocessRGBCameras
from right_pick.fast_live_loop import FastLiveClosedLoop,verify_preflight_contract
p=Path(__file__).resolve().parent;c=json.loads((p/'fast_live_commissioned.json').read_text());f=json.loads((p/'fast_codex_120.json').read_text())
prior=Path('/home/agilex/piper_right_pick_demo/runs/astra_fast_physical/20261005T111651Z_3dde582d04fc');previous=json.loads((prior/'fast_codex_output_0001.json').read_text())['decision']
assert previous['phase']=='APPROACH_PEN' and previous['action']=='move_eef'
r=Recorder('/home/agilex/piper_right_pick_demo/runs/astra_fast_physical',{'site':c,'fast':f},'Resume the single right-arm pen-to-holder RGB-only task after a model-requested pause; no object geometry or calibration.',model_id='gpt-6-astra',mode='physical')
a=ROSRightArm(c,recorder=r,proposal_only=False);cams=None
try:
 ready=verify_preflight_contract(a.preflight());r.event('resume_preflight',ready)
 m=CodexDecisionClient(f['model'],r);cams=SubprocessRGBCameras(c,r.run_dir/'observations')
 import time
 warmup_started=time.time();cams.capture();time.sleep(3);r.event('camera_startup_warmup',{'elapsed_s':time.time()-warmup_started,'purpose':'Let automatic exposure settle; loop takes new current RGB afterward','excluded_from_fast_loop_elapsed':True})
 loop=FastLiveClosedLoop(model=m,robot=a,cameras=cams,recorder=r,limits=c['physical_limits'],options=f['controller'])
 loop.phase='APPROACH_PEN';loop.retry_count=1;loop.previous_action=previous;loop.previous_result={'status':'rejected','error_code':'encoded_translation_exceeded_30mm_before_dispatch'}
 loop.memory='Front faces robot (right=image-left). Previous 30mm endpoint was rejected before send: ROS rounds XYZ to 1mm, exceeding 30mm. Leave at least 1mm margin below the action budget. Reassess fresh RGB.'
 r.event('resume_after_presend_rejection',{'previous_run':str(prior),'phase':loop.phase,'retry_count':1,'no_previous_physical_task_actions':True,'visual_judgment_supplied_by_host':False,'user_correction':'Right gripper is in third-person image','view_convention':'Front faces robot; anatomical right corresponds to image-left','independently_autonomous_trial':False,'prompt_revision':'Assess concrete next bounded motion rather than requiring every tool surface visible'})
 print(json.dumps({'event':'resumed','run_dir':str(r.run_dir)}),flush=True)
 result=loop.run();print(json.dumps(result,indent=2),flush=True)
finally:
 if cams is not None:cams.close()
 a.close()
