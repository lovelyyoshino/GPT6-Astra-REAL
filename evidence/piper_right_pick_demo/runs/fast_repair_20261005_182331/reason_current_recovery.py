import json,time
from pathlib import Path
from right_pick.recording import Recorder
from right_pick.fast_recording import FastRunMetrics
from right_pick.fast_ros import ROSRightArm
from right_pick.fast_codex import CodexDecisionClient
from right_pick.fast_observation import SubprocessRGBCameras
from right_pick.fast_policy import compact_controller_state,parse_response,validate_decision
p=Path(__file__).resolve().parent;c=json.loads((p/'fast_live_commissioned.json').read_text());f=json.loads((p/'fast_codex_120.json').read_text())
r=Recorder('/home/agilex/piper_right_pick_demo/runs/astra_current_reasoning',{'site':c,'fast':f},'Reassess current RGB and measured right-arm state after the previous P trajectory anomaly. Exactly one next-step proposal and short explanation. No command dispatch.',model_id='gpt-6-astra',mode='physical');metrics=FastRunMetrics(r);a=ROSRightArm(c,recorder=r,proposal_only=True);cams=SubprocessRGBCameras(c,r.run_dir/'observations');m=CodexDecisionClient(f['model'],r)
row=dict(step_id=1,phase='RECOVERY',timestamp=time.time(),model_request_start=None,model_response_end=None,agent_decide_s=0.,image_capture_s=0.,image_encode_s=0.,robot_execute_s=0.,robot_wait_s=0.,total_step_s=0.,selected_camera_views=[],retry_count=2,action=None,action_arguments=None,confidence=None,decision_requested=False,action_dispatched=False,dispatch_attempted=False,decision_source='codex_cli',phase_transition={'from':'RECOVERY','to':'RECOVERY'},execution_scope='current_sensor_model_proposal_only')
started=time.monotonic();passed=False
try:
 m.prepare();a.observe();tick=time.monotonic();cams.capture();time.sleep(3);o=cams.capture();row['image_capture_s']=time.monotonic()-tick;s=a.observe()
 prior=Path('/home/agilex/piper_right_pick_demo/runs/astra_fast_physical/20261005T111800Z_4c757c312206');previous=json.loads((prior/'fast_codex_output_0001.json').read_text())['decision']
 state=compact_controller_state({'phase':'RECOVERY','robot_state':s['robot_state'],'gripper_state':s['gripper_state'],'previous_action':previous,'previous_result':{'status':'failed','error_code':'P_path_bound_exceeded_then_goal_arrived'},'retry_count':2,'memory':'Front faces robot; right=image-left. Last +28mm P arrived but J4 changed 69deg and path monitoring failed. Reassess current RGB and next action. XYZ rounds to 1mm; leave margin. Proposal only.','action_budget':{'max_translation_m':.0045,'max_rotation_rad':.0075,'max_speed_percent':1,'max_waypoints':1,'gripper_min_m':0.,'gripper_max_m':.055,'max_effort_parameter_nm':.2,'allow_waypoint_chunks':False,'required_effort_parameter_nm':.2}})
 r._write_json('current_controller_state.json',state);r._write_json('current_observation.json',o);r._write_json('current_robot_feedback.json',s);row['previous_result']=state['previous_result'];row['decision_requested']=True
 print(json.dumps({'event':'model_request_started','run_dir':str(r.run_dir),'effort':'xhigh','commands_enabled':False}),flush=True)
 try:proposal=m.decide(state,o)
 finally:
  for key,value in m.last_metrics.items():
   if key in row or key in ('reasoning_effort','requested_model','actual_model','input_tokens','output_tokens','reasoning_output_tokens','request_id'):row[key]=value
 r._write_json('next_step_proposal.json',proposal);d=parse_response(proposal,require_explanation=True);row.update(action=d.action,action_arguments=d.arguments,confidence=d.confidence);validate_decision(d,'RECOVERY',limits=c['physical_limits'],controller_state=state);passed=True
 r._write_json('post_reasoning_robot_feedback.json',a.observe());print(json.dumps({'proposal':proposal,'metrics':m.last_metrics,'dispatched':False},ensure_ascii=False,indent=2),flush=True)
except Exception as exc:
 row['error']=type(exc).__name__;r.event('diagnostic_error',{'type':type(exc).__name__,'message':str(exc)});print(json.dumps({'error':str(exc),'run_dir':str(r.run_dir)}),flush=True)
finally:
 cams.close();a.close();row['total_step_s']=time.monotonic()-started;metrics.step(row);report=metrics.finish(termination_reason='current_recovery_reasoning_complete' if passed else 'current_recovery_reasoning_failed',phase='RECOVERY',nonphysical=False,live_check=True,live_check_passed=passed);print(json.dumps({'summary':report,'run_dir':str(r.run_dir)},indent=2),flush=True)
