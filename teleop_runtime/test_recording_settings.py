"""Persistent recording selection and safe private camera profiles."""
import json
from pathlib import Path
import pytest
from teleop_runtime import recording_settings as settings


def test_defaults_persist_and_reject_invalid_options(tmp_path,monkeypatch):
    monkeypatch.setenv('COLLECTION_RECORDING_CONFIG',str(tmp_path/'recording.json'))
    assert settings.read_settings()==dict(format='mcap',head_depth=True,wrist_depth=True,tactile=False)
    settings.update_settings(format='rosbag',head_depth=False,tactile=True)
    assert settings.read_settings()==dict(format='rosbag',head_depth=False,wrist_depth=True,tactile=True)
    with pytest.raises(ValueError):settings.update_settings(tactile='false')
    assert settings.read_settings()['tactile'] is True


def test_camera_selection_uses_private_profiles(tmp_path,monkeypatch):
    import yaml
    monkeypatch.setenv('COLLECTION_RECORDING_CONFIG',str(tmp_path/'prefs/recording.json'))
    params=tmp_path/'original.yaml';params.write_text('enable_depth: false\ncolor_width: 640\ncolor_height: 480\n')
    config=tmp_path/'cameras.yaml';config.write_text(yaml.safe_dump({'camera_bringup':dict(log_level='none',**{c:dict(config_file=str(params)) for c in ('cam0','cam1','cam2')})}))
    before=params.read_bytes();settings.update_settings(head_depth=False,wrist_depth=True)
    result=yaml.safe_load(settings.camera_config(config).read_text())
    for c,entry in result['camera_bringup'].items():
        if c=='log_level':continue
        actual=yaml.safe_load(Path(entry['config_file']).read_text())
        assert actual['enable_depth']==(c!='cam0')
        assert actual['color_width']==640
        if c!='cam0':assert (actual['depth_width'],actual['depth_height'],actual['depth_fps'])==(640,480,30)
    assert params.read_bytes()==before
    old_path=settings.camera_config(config);old_bytes=old_path.read_bytes()
    settings.update_settings(head_depth=True)
    new_path=settings.camera_config(config)
    assert old_path!=new_path and old_path.read_bytes()==old_bytes


def test_task_descriptions_survive_task_edits(tmp_path,monkeypatch):
    from operator_tasks import set_task_description,task_description,add_operator_task,get_operator_tasks
    monkeypatch.setenv('OPERATOR_TASKS_FILE',str(tmp_path/'tasks.json'))
    set_task_description('assembly','Assemble the parts.')
    added=add_operator_task('测试')
    assert task_description('assembly')=='Assemble the parts.'
    assert task_description(added)==''
    assert added in get_operator_tasks()
