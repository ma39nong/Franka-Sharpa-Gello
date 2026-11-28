"""Atomic local metadata writes, independent of hardware."""
import json
import os
from pathlib import Path
import tempfile

def write_json(path, value):
    path=Path(path);temporary=None
    try:
        with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',dir=path.parent,prefix='.'+path.name,delete=False) as stream:
            temporary=Path(stream.name);json.dump(value,stream,ensure_ascii=False,indent=2);stream.write('\n');stream.flush();os.fsync(stream.fileno())
        temporary.replace(path)
    finally:
        if temporary:temporary.unlink(missing_ok=True)


def recorder_commit():
    import subprocess
    root=Path(__file__).resolve().parents[4]
    return subprocess.check_output(['git','-c','safe.directory='+str(root),'-C',str(root),'rev-parse','HEAD'],text=True).strip()
