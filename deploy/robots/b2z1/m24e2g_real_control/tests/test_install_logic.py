import importlib.util
from pathlib import Path
import unittest
import yaml

BASE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('installer',BASE/'tools/install_overlay.py')
ins=importlib.util.module_from_spec(spec);spec.loader.exec_module(ins)

class InstallerTests(unittest.TestCase):
    def test_anchors_and_output(self):
        main='''#include "State_InteractiveArmRealPreview.h" // M2.4e2f preview ONLY
void init_fsm_state(){
    FSMState::lowstate = std::make_shared<LowState_t>();
}
int main(){
    unitree::robot::ChannelFactory::Instance()->Init(0, network);
}
'''
        cm='''include_directories(
  ${PROJECT_SOURCE_DIR}/m24e2f_real_fsm_preview/include
)
target_sources(${PROJECT_NAME}_lib PRIVATE
  ${PROJECT_SOURCE_DIR}/m24e2f_real_fsm_preview/src/State_InteractiveArmRealPreview.cpp
)
'''
        fsm='''void pre_run(){
        lowstate->update();
#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK
        if(keyboard) {
            keyboard->update();
            if(keyboard_joystick) keyboard_joystick->update(lowstate->joystick);
        }
        update_z1_diagnostic_toggle();
#else
        if(keyboard) {
            keyboard->update();
            KeyboardJoystick::update_joystick(lowstate->joystick, keyboard->key());
        }
#endif
}
    static int lowcmd_joint_count_for_whole_body()
    {
}
'''
        got=ins.patch({ins.SRCMAIN:main,ins.CMAKE:cm,ins.FSM:fsm})
        self.assertIn('StampedB2LowState',got[ins.SRCMAIN]);self.assertIn('return 12;',got[ins.FSM]);
        self.assertIn('REMOTE_DATA_RX key{}',got[ins.FSM]);
        self.assertIn('m24e2g_real_control/src/State_InteractiveArmReal.cpp',got[ins.CMAKE]);
        self.assertEqual(ins.patch(got),got)
    def test_yaml_is_12dof_and_keyboard_only(self):
        stand=[.1,.8,-1.5]*4+[0,1.48,-.63,-.84,0,1.57]
        conf={'FSM':{
            '_':{k:{'id':i} for i,k in enumerate(['Passive','FixStand','SlowLieDown'],1)},
            'Passive':{'transitions':{},'kp':[0]*18,'kd':[3]*18,'mode':[1]*18,'target_indices':list(range(12,18)),'target_q':[0]*6,'target_duration':2},
            'FixStand':{'transitions':{},'kp':[500]*18,'kd':[6]*18,'ts':[0,2],'qs':[[],stand]},
            'SlowLieDown':{'transitions':{},'kp':[350]*18,'kd':[6]*18,'ts':[0,5],'qs':[[],stand]},
        },'z1_sdk':{'enabled':True},'z1_diagnostic':{'enabled':True}}
        after=yaml.safe_load(ins.config(yaml.safe_dump(conf)))
        self.assertEqual(after['FSM']['FixStand']['qs'][1],stand[:12]);
        self.assertEqual(len(after['FSM']['SlowLieDown']['qs'][1]),12)
        self.assertEqual(after['FSM']['FixStand']['transitions']['InteractiveArmReal'],'X.on_pressed')
        self.assertFalse(after['z1_sdk']['enabled']);self.assertFalse(after['z1_diagnostic']['enabled'])
        self.assertTrue(after['interactive_real']['keyboard_only'])
        self.assertFalse(after['interactive_real']['enable_b2_output'])
        self.assertFalse(after['interactive_real']['enable_z1_sdk_targets'])

class MockJointConfigTests(unittest.TestCase):
    def test_mock_j5_only_config(self):
        conf=yaml.safe_load((BASE.parents[0]/'config/config_m24e2g_mock_joint_jog_sim.yaml').read_text())
        i=conf['interactive_real']
        self.assertTrue(i['enabled'])
        self.assertEqual(i['z1_backend'],'mock')
        self.assertTrue(i['enable_mock_z1_j5_jog'])
        self.assertFalse(i['enable_b2_output'])
        self.assertFalse(i['enable_z1_sdk_targets'])
        self.assertFalse(i['enable_walking'])
        self.assertFalse(conf['z1_sdk']['enabled'])
        self.assertFalse(conf['z1_diagnostic']['enabled'])
        self.assertEqual(conf['FSM']['_']['InteractiveArmReal']['type'],'InteractiveArmReal')


class SourceGuards(unittest.TestCase):
    def test_timestamps_locked_and_motor_scope(self):
        base=(BASE/'include/StampedB2LowState.h').read_text()
        state=(BASE/'src/State_InteractiveArmReal.cpp').read_text()
        self.assertIn('void post_communication() override',base)
        self.assertIn('Clock::now()',base)
        self.assertIn('std::lock_guard<std::mutex> lk(lowstate->mutex_)',state)
        self.assertIn('stamped->receivedAtLocked()',state)
        self.assertIn('for(int i=0;i<12;++i)',state)
        self.assertNotIn('motor_cmd()[12',state)
        self.assertNotIn('setArmCmd(',state)
        self.assertIn('enable_z1_sdk_targets',state)
        self.assertIn('registered_checks.emplace_back',state)

if __name__=='__main__': unittest.main(verbosity=2)
