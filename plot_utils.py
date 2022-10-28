#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 21 19:28:26 2021
3D arrows taken from https://gist.github.com/WetHat/1d6cd0f7309535311a539b42cccca89c 

@author: Natalia Paredes
"""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.proj3d import proj_transform
from mpl_toolkits.mplot3d.axes3d import Axes3D
from matplotlib.patches import FancyArrowPatch
import matplotlib.animation as anim
import numpy as np

class Arrow3D(FancyArrowPatch):

    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)

    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)
        
def _arrow3D(ax, x, y, z, dx, dy, dz, *args, **kwargs):
    '''Add an 3d arrow to an `Axes3D` instance.'''

    arrow = Arrow3D(x, y, z, dx, dy, dz, *args, **kwargs)
    ax.add_artist(arrow)

setattr(Axes3D, 'arrow3D', _arrow3D)

def plotTrans(H_list):
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    ax.set_xlim(0.0,0.8)
    ax.set_ylim(-0.4,0.4)
    ax.set_zlim(0,0.8)
    ax.set_yticks([])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.view_init(0,270)
    arrow_length = 0.15
    coords = []
    
    #plot world coord-frame
    ax.plot(0,0,0,'o',markersize=5,color='black')  
    #coords.append([0,0,0])
    ax.arrow3D(0,0,0,0,0,arrow_length,mutation_scale=5,ec ='blue')
    ax.arrow3D(0,0,0,0,arrow_length,0,mutation_scale=5,ec ='green')
    ax.arrow3D(0,0,0,arrow_length,0,0,mutation_scale=5,ec ='red')
    ax.text(-0.02,-0.1,-0.1,'$w$',size=10, math_fontfamily='dejavuserif')
    
    #plot the other coord-frames
    for i,H in enumerate(H_list):
        ax.plot(H[0,3],H[1,3],H[2,3],'o',markersize=5,color='black') 
        
        coords.append([H[0,3],H[1,3],H[2,3]])
        x = np.matmul(H[0:3,0:3],np.array([[arrow_length],[0],[0]]))
        y = np.matmul(H[0:3,0:3],np.array([[0],[arrow_length],[0]]))
        z = np.matmul(H[0:3,0:3],np.array([[0],[0],[arrow_length]]))
        
        ax.arrow3D(H[0,3],H[1,3],H[2,3],x[0][0],x[1][0],x[2][0],mutation_scale=5,ec ='red')
        ax.arrow3D(H[0,3],H[1,3],H[2,3],y[0][0],y[1][0],y[2][0],mutation_scale=5,ec ='green')
        ax.arrow3D(H[0,3],H[1,3],H[2,3],z[0][0],z[1][0],z[2][0],mutation_scale=5,ec ='blue')
        ax.text(H[0,3]-0.015,H[1,3]-0.17,H[2,3]+0.02,'$'+str(i)+'$',color='black', size=10, math_fontfamily='dejavuserif')
        
        #ax.plot([H[0,3],0],[H[1,3],H[1,3]],[H[2,3],H[2,3]],linestyle='dashed',color='gray',linewidth='0.8') 
        #ax.plot([H[0,3],H[0,3]],[H[1,3],0],[H[2,3],H[2,3]],linestyle='dashed',color='gray',linewidth='0.8') 
        ax.plot([H[0,3],H[0,3]],[H[1,3],H[1,3]],[H[2,3],0],linestyle='dashed',color='gray',linewidth='0.8') 
    
    coords = np.asarray(coords)
    ax.plot(coords[:,0],coords[:,1],coords[:,2],linestyle='solid',color='black',linewidth='0.8')
    #plt.tight_layout()
    fig.savefig(str('test.pdf'), format='pdf', bbox_inches='tight')
    
def plotTrans_anim(H_lamb,q1_,q2_,q3_,L1_,L2_,L3_,fig,ax,init_plot=False):
    #fig = plt.figure()
    #ax = fig.add_subplot(111,projection='3d')
    
    arrow_length = 0.2
    coords = []
    
    #plot world coord-frame
    ax.plot(0,0,0,'o',markersize=5,color='black')  
    #coords.append([0,0,0])
    ax.arrow3D(0,0,0,0,0,arrow_length,mutation_scale=5,ec ='blue')
    ax.arrow3D(0,0,0,0,arrow_length,0,mutation_scale=5,ec ='green')
    ax.arrow3D(0,0,0,arrow_length,0,0,mutation_scale=5,ec ='red')
    ax.text(-0.02,-0.1,-0.1,'$w$',size=10, math_fontfamily='dejavuserif')
    
    #plot the other coord-frames
    for i,Hlamb in enumerate(H_lamb):
        H = Hlamb(q1_,q2_,q3_,L1_,L2_,L3_,np.pi)
        ax.plot(H[0,3],H[1,3],H[2,3],'o',markersize=5,color='black') 
        
        coords.append([H[0,3],H[1,3],H[2,3]])
        x = np.matmul(H[0:3,0:3],np.array([[arrow_length],[0],[0]]))
        y = np.matmul(H[0:3,0:3],np.array([[0],[arrow_length],[0]]))
        z = np.matmul(H[0:3,0:3],np.array([[0],[0],[arrow_length]]))
        
        ax.arrow3D(H[0,3],H[1,3],H[2,3],x[0][0],x[1][0],x[2][0],mutation_scale=5,ec ='red')
        ax.arrow3D(H[0,3],H[1,3],H[2,3],y[0][0],y[1][0],y[2][0],mutation_scale=5,ec ='green')
        ax.arrow3D(H[0,3],H[1,3],H[2,3],z[0][0],z[1][0],z[2][0],mutation_scale=5,ec ='blue')
        ax.text(H[0,3]-0.015,H[1,3]-0.17,H[2,3]+0.09,'$'+str(i)+'$',color='black', size=10, math_fontfamily='dejavuserif')
        
        #ax.plot([H[0,3],0],[H[1,3],H[1,3]],[H[2,3],H[2,3]],linestyle='dashed',color='gray',linewidth='0.8') 
        #ax.plot([H[0,3],H[0,3]],[H[1,3],0],[H[2,3],H[2,3]],linestyle='dashed',color='gray',linewidth='0.8') 
        ax.plot([H[0,3],H[0,3]],[H[1,3],H[1,3]],[H[2,3],0],linestyle='dashed',color='gray',linewidth='0.8') 
    
    coords = np.asarray(coords)
    ax.plot(coords[:,0],coords[:,1],coords[:,2],linestyle='solid',color='black',linewidth='0.8')

    #plt.tight_layout()
    #fig.savefig(str('test.pdf'), format='pdf', bbox_inches='tight')
    
def plot_cont(fun, xmax):
    y = []
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)

    def update(i):
        yi = fun()
        y.append(yi)
        x = range(len(y))
        ax.clear()
        ax.plot(x, y)
        print (i, ': ', yi)

    a = anim.FuncAnimation(fig, update, frames=xmax, repeat=False)
    plt.show()