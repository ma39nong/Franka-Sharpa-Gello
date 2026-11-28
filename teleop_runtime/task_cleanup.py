"""One-shot task metadata editing; initializes no ROS nodes or devices."""
import argparse
import json
import os
from operator_tasks import add_operator_task,delete_operator_task,set_task_description


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('add','delete','describe'))
    parser.add_argument('value')
    args=parser.parse_args()
    os.environ['TELEOP_TASK_HELPER']='1'
    if args.action=='describe':
        value=json.loads(args.value)
        result=set_task_description(value['task'],value['description'])
    else:result=(add_operator_task(args.value) if args.action=='add' else delete_operator_task(args.value))
    print(json.dumps(dict(result=result),ensure_ascii=False))

if __name__=='__main__':main()
