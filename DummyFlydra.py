import numpy as np
import time
pi = np.pi

# create a dummy mainbrain with arbitrary number of flies, flies are born and killed, and move around

class DummyMainbrain:

    def __init__(self, max_num_flies = 5):
        self.max_num_flies = max_num_flies
        self.latency = 0.05 # 50 msec

        nflies = np.random.randint(0,self.max_num_flies)
        self.flies = [DummyFly() for i in range(nflies)]

        self.prob_birth = 0.02
        self.prob_death = 0.02

    def get_flies(self):
        time.sleep(self.latency)

        birth_check = np.random.random()
        if birth_check < self.prob_birth:
            if len(self.flies) < self.max_num_flies:
                self.flies.append(DummyFly())

        death_check = np.random.random()
        if death_check < self.prob_death:
            if len(self.flies) > 1:
                del self.flies [np.random.randint(0,len(self.flies))]

        obj_ids = np.zeros(len(self.flies), dtype=np.uint16)
        state_vecs = np.zeros([len(self.flies), 6])
        for i in range(len(self.flies)):
            obj_ids[i] = self.flies[i].get_obj_id()
            state_vecs[i,:] = self.flies[i].get_state_vecs()

        return self.latency, obj_ids, state_vecs
    

class DummyFly:

    def __init__(self):
        # random start position values
        self.x0 = np.random.randn()
        self.y0 = np.random.randn()
        self.z0 = np.random.randn()

        # random amplitude values
        self.xamp = np.random.randn()
        self.yamp = np.random.randn()
        self.zamp = np.random.randn()

        # object ID number
        self.obj_id = np.random.randint(0,100)

    def get_state_vecs(self):
        theta = time.time() % (2*pi)
        x = self.xamp*np.cos( theta ) + self.x0
        xvel = -self.xamp*np.sin( theta )
        y = self.yamp*np.sin( theta ) + self.y0
        yvel = -self.yamp*np.cos( theta )
        z = self.zamp*np.sin( theta/2.3 ) + self.z0
        zvel = -self.zamp/2.3*np.cos( theta/2.3 )   

        state_vecs = [x,y,z,xvel,yvel,zvel]

        return state_vecs

    def get_obj_id(self):
        return self.obj_id



########### dummy reprojection ###############

def reproject(fly_pos, screensize = [640,480]):

    xpos = int(np.abs(fly_pos[0]/2.*screensize[0]))
    ypos = int(np.abs(fly_pos[1]/2.*screensize[1]))

    return xpos, ypos








    
