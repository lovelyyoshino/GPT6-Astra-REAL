"""Pure contiguous-suffix stability window; unchanged feedback requirements.

Only an oldest prefix is removed. No internal sample is skipped. All retained
old pairs were already valid, so comparing each new orientation to every old
orientation preserves the original all-pairs invariant without quadratic work.
"""
import copy
import math
from ros_home_step import StableWindow as FrozenStableWindow, require, rotation


class BaselineNotReady(RuntimeError):
    """Only a timed-out, health-checked read-only baseline may raise this."""


class StableWindow(FrozenStableWindow):
    def add(self,s,monotonic):
        if self.samples:
            previous=self.samples[-1][1]
            require(s['sequence']>previous['sequence']
                and all(a>b for a,b in zip(s['stamps'],previous['stamps'])),
                'All fourteen feedback fragments must advance')
            require(monotonic>=self.samples[-1][0],'Monotonic sampling time regressed')
        self.samples.append((monotonic,copy.deepcopy(s)))
        while len(self.samples)>1 and monotonic-self.samples[1][0]>=3.:
            self.samples.popleft()
        while len(self.samples)>1 and not self._valid(s):
            self.samples.popleft()
        return len(self.samples)>=20 and monotonic-self.samples[0][0]>=3.

    def _valid(self,newest):
        rows=[s for _,s in self.samples]
        if any(max(r['q'][i]for r in rows)-min(r['q'][i]for r in rows)>.003 for i in range(6)):
            return False
        xyz=[max(r['pose'][i]for r in rows)-min(r['pose'][i]for r in rows)for i in range(3)]
        return (math.sqrt(sum(v*v for v in xyz))<=.0005
            and max(r['opening_m']for r in rows)-min(r['opening_m']for r in rows)<=.0005
            and all(rotation(newest['pose'],r['pose'])<=.003 for r in rows))
