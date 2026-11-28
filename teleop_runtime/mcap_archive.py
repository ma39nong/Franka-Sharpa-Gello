"""Read-only archive identity and relocatable asset verification for host transfers."""
import hashlib
import json
from pathlib import Path

OWNER='gello_upper_body_teleop'


def digest(path):
    path=Path(path);h=hashlib.sha256()
    if path.is_file():
        with path.open('rb') as stream:
            for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    else:
        for p in sorted(path.rglob('*')):
            if p.is_file():h.update(p.relative_to(path).as_posix().encode()+b'\0'+digest(p).encode()+b'\n')
    return h.hexdigest()


def collection_state(source):
    source=Path(source)
    if (source/'collection_state.json').is_file():
        return json.loads((source/'collection_state.json').read_text())
    document=json.loads((source/'episode.json').read_text())
    state=document.get('collection',{})
    if state.get('owner')!=OWNER or document.get('recorder',{}).get('name')!='gello-mcap':
        raise ValueError('仅处理本工程新采集的样本')
    return state


def assets(source):
    """Return only immutable referenced assets under this archive root."""
    source=Path(source).resolve();root=source.parent.parent
    if not (source/'data.mcap').is_file():return []
    document=json.loads((source/'episode.json').read_text())
    result=[]
    for key,ref in document['assets'].items():
        lexical=source/ref['path'];path=lexical.resolve(strict=True)
        if not path.is_relative_to(root/'_shared') and not path.is_relative_to(root/'assets/models'):
            raise ValueError('共享资产引用超出归档根目录')
        for part in [lexical,*lexical.parents]:
            if part.is_symlink():raise ValueError('共享资产不能是符号链接')
            if part==root:break
        if digest(path)!=ref['sha256']:raise ValueError('共享资产哈希不符：'+key)
        # Move complete hashed shared versions together, preserving all references.
        relative=path.relative_to(root)
        version=root.joinpath(*relative.parts[:2]) if relative.parts[0]=='_shared' else path
        if version not in result:result.append(version)
    return result
