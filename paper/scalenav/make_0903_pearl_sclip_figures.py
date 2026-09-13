from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

ROOT=Path('scalenav_ws/tmp/0903_replay')
OUT=Path('paper/scalenav/pics/candidates/0903_pearl_vs_sclip')
OUT.mkdir(parents=True,exist_ok=True)
def heat(a):
 a=np.asarray(a,float); lo,hi=np.percentile(a,[2,98]); return np.clip((a-lo)/max(hi-lo,1e-9),0,1)
for run in sorted(ROOT.glob('run_*/rgb_capture.jpg')):
 key=run.parent.name; s=np.load(OUT/(key+'_sclip.npy'))
 rgb=np.asarray(Image.open(run).convert('RGB'))
 pearl=Image.open(run.parent/'pearl_overlay.jpg').convert('RGB')
 fig,ax=plt.subplots(1,4,figsize=(15,3.7),constrained_layout=True)
 ax[0].imshow(rgb); ax[0].set_title('0903 real-flight RGB')
 ax[1].imshow(pearl); ax[1].set_title('PEARL overlay')
 ax[2].imshow(rgb); ax[2].imshow(heat(s),cmap='turbo',alpha=.62,vmin=0,vmax=1); ax[2].set_title('SCLIP overlay')
 pa=np.asarray(pearl.resize((s.shape[1],s.shape[0])),float).mean(2)/255
 ax[3].imshow(np.abs(heat(s)-pa),cmap='magma',vmin=0,vmax=1); ax[3].set_title('|SCLIP − PEARL| (proxy)')
 for a in ax:a.axis('off')
 fig.savefig(OUT/(key+'.png'),dpi=220,bbox_inches='tight'); plt.close(fig)
print('wrote',len(list(OUT.glob('run_*.png'))),'figures to',OUT)
