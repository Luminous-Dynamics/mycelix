#!/usr/bin/env python3
import argparse,pathlib,sys
ROOT=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
import spec,guard,product,execute
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--repo',type=pathlib.Path,default=ROOT.parents[1]);p.add_argument('--receipt',type=pathlib.Path,required=False);a=p.parse_args()
 if a.receipt:execute.qualify(a.repo,a.receipt,spec,guard,product,{'qualify.py':__file__,'spec.py':spec.__file__,'guard.py':guard.__file__,'product.py':product.__file__,'execute.py':execute.__file__})
