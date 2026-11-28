"""Qt settings, persistence, and task snapshot without hardware."""
import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from pathlib import Path
import pytest
from PySide6.QtWidgets import QApplication
from apps.operator_gui.operator_gui import OperatorWindow


def test_recording_controls_remember_and_freeze(tmp_path,monkeypatch):
    monkeypatch.setenv('COLLECTION_RECORDING_CONFIG',str(tmp_path/'recording.json'))
    monkeypatch.setenv('OPERATOR_TASKS_FILE',str(tmp_path/'tasks.json'))
    app=QApplication.instance() or QApplication([])
    window=OperatorWindow('127.0.0.1',1,collection_port=2)
    try:
        assert window._recording_options()==dict(format='mcap',head_depth=True,wrist_depth=True,tactile=False)
        window.recording_format.setCurrentIndex(window.recording_format.findData('rosbag'))
        window.recording_sensor_boxes['head_depth'].setChecked(False)
        window.task_description_edit.setText('Assemble the parts.')
        window._save_task_description()
        window._apply_collection_status(dict(state='RECORDING',active=True,can_stop=True))
        assert not window.recording_format.isEnabled()
        assert not window.task_description_edit.isEnabled()
        assert not window.task_selector.isEnabled()
        window._apply_collection_status(dict(state='READY',active=False,can_start=True))
        assert window.recording_format.isEnabled()
    finally:window.close();app.processEvents()
    restored=OperatorWindow('127.0.0.1',1,collection_port=2)
    try:
        assert restored._recording_options()['format']=='rosbag'
        assert restored._recording_options()['head_depth'] is False
        assert restored.task_description_edit.text()=='Assemble the parts.'
    finally:restored.close();app.processEvents()
