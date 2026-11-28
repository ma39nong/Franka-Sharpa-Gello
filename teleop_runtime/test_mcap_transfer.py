"""MCAP uploads include referenced assets; foreign archives are never managed."""
import json
from pathlib import Path
import pytest
from teleop_runtime.mcap_archive import digest,assets
from teleop_runtime.transfer_worker import Operation,ready_state
from teleop_runtime.collection_transfer import Watcher
from teleop_runtime.transfer_settings import atomic_json
from teleop_runtime.test_collection_transfer import fake_mount_identity


def make_episode(root,number=1,owner='gello_upper_body_teleop'):
    path=root/'A'/f'episode{number}';path.mkdir(parents=True)
    shared=root/'_shared'/'config-hash';shared.mkdir(parents=True,exist_ok=True)
    (shared/'devices.yaml').write_text('devices: []')
    for name in ('calibration','configs'):
        (shared/name).mkdir(exist_ok=True);(shared/name/'info.yaml').write_text('software_fixture: true')
    model=root/'assets/models/model-hash';model.mkdir(parents=True,exist_ok=True);(model/'robot.urdf').write_text('<robot name="test"/>')
    refs={k:dict(path=str(p.relative_to(root)),sha256=digest(p)) for k,p in dict(model_ref=model,calibration_ref=shared/'calibration',devices_ref=shared/'devices.yaml',config_ref=shared/'configs').items()}
    for ref in refs.values():ref['path']='../../'+ref['path']
    (path/'data.mcap').write_bytes(b'software fixture')
    atomic_json(path/'episode.json',dict(recording_id='id-1',recorder=dict(name='gello-mcap' if owner=='gello_upper_body_teleop' else 'sharpa-mcap'),assets=refs,
        collection=dict(owner=owner,source_recording_id='id-1',quality='优等',storage=dict(state='saved',folder='A'),transfer_ready=True)))
    return path


def test_upload_verifies_assets_and_preserves_relative_references(tmp_path,monkeypatch):
    source=make_episode(tmp_path/'local');remote=tmp_path/'remote';remote.mkdir()
    monkeypatch.setenv('COLLECTION_TRANSFER_CONFIG',str(tmp_path/'prefs.json'))
    monkeypatch.setenv('COLLECTION_TRANSFER_STATE',str(tmp_path/'state'))
    monkeypatch.setattr('teleop_runtime.transfer_worker.mount_for',lambda _:fake_mount_identity(remote))
    work=tmp_path/'job';work.mkdir()
    operation=Operation(dict(id='test',source=str(source),folder='A',recording_id='id-1',remote_parent=str(remote),rate_mib=1000),work)
    proof=operation.upload()
    assert proof['assets']
    uploaded=remote/'A/episode1'
    assert assets(uploaded)
    assert {p.name for p in uploaded.iterdir()}=={p.name for p in source.iterdir()}
    assert (remote/'.upload-receipts/A/episode1.json').is_file()
    assert ready_state(source)['source_recording_id']=='id-1'
    (remote/'_shared/config-hash/devices.yaml').write_text('corruption')
    with pytest.raises(RuntimeError,match='资产'):operation.verify_upload(operation.copy_upload())
    assert source.is_dir()


def test_shared_folder_watcher_ignores_dagger_and_historical(tmp_path,monkeypatch):
    monkeypatch.setenv('COLLECTION_TRANSFER_CONFIG',str(tmp_path/'prefs.json'))
    root=tmp_path/'local';make_episode(root)
    config=dict(local_parent=str(root),remote_parent='',upload_enabled=True,delete_enabled=True)
    watcher=Watcher(settle=0);assert watcher.scan(config,0)==[]
    foreign=make_episode(root,2,owner='dagger')
    own=make_episode(root,3)
    watcher.scan(config,1);jobs=watcher.scan(config,2)
    assert [j['source'] for j in jobs]==[str(own)]
    with pytest.raises(ValueError):ready_state(foreign)
    assert foreign.is_dir()
