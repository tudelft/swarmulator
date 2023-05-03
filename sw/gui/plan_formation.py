import matplotlib.pyplot as plt

class Graphic(object):

    def init_func(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        # axis dimensions
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])

        # uav_list
        self.uavs = []
        self.di = list()  # np.zeros((N,N))
        
        # generate meshgrid

    # function to be followed on every button click
    def onclick(self,event):

        self.uavs.append([event.ydata, event.xdata])
        # unzip x_y
        temp_y, temp_x = zip(*self.uavs)
        plt.plot(temp_x, temp_y)
        plt.plot(event.xdata, event.ydata, color='red', marker='o', markersize=9)

        self.fig.canvas.draw()


    def display(self):
        self.init_func()
        # plt.scatter(self.x, self.y)
        x, y, arrow_length = 0.5, 0.5, 0.15
        self.ax.annotate('N', xy=(x, y), xytext=(x, y-arrow_length),
            arrowprops=dict(facecolor='red', width=5, headwidth=15),
            ha='center', va='center', fontsize=20,
            xycoords=self.ax.transAxes)
        cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)

        plt.show()
        # d_matrix = distance.cdist(self.uavs, self.uavs, 'euclidean')
        N = len(self.uavs)
        # N x N x 2  array 
        v_mat = [[[0.,0.] for _ in range(N)] for _ in range(N)]
        for i in range(N):
            for j in range(i):
                v_mat[i][j] = [self.uavs[i][0] - self.uavs[j][0],self.uavs[i][1] - self.uavs[j][1]]
                v_mat[j][i] = [self.uavs[j][0] - self.uavs[i][0],self.uavs[j][1] - self.uavs[i][1]]

        if plt.close() == True:
            print("OKAY")
        plt.close()
        return v_mat


if __name__=='__main__':
    formation  = Graphic().display()
    print("Writing formation..")

    with open("./conf/formations/formation.csv", "w") as file:
        file.write(f"{len(formation)} {len(formation[0])} {len(formation[0][0])}")
        for row in formation:
            temp = "\n"
            for col in row:
                temp+=f"{col[0]:.3f},{col[1]:.3f} "
            file.write(temp[:-1])
            # file.seek(-1)
            # file.truncate()
