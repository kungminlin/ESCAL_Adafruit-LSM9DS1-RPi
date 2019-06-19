import matplotlib.pyplot as plt
import matplotlib.animation as animation
import datetime

class Dataplot:
	def __init__(self, fig):
		self.fig = fig
		self.plots = {}

	def add_subplot(self, name, title):
		ax = self.fig.add_subplot(1, 1, 1)
		x = []
		y = []
		self.plots[name] = {'title': title, 'ax': ax, 'x': x, 'y': y}

	def get_plots(self):
		return self.plots

	def plot(self, name, data):
		title = self.plots[name].title
		ax = self.plots[name].ax
		x = self.plots[name].x
		y = self.plots[name].y

		x.append(datetime.datetime.now().strftime('%H:%M:%S:%f'))
		y.append(data)

		x = x[-20:]
		y = y[-20:]

		ax.clear()
		ax.plot(x, y)
		ax.set_title(title)

	def animate(self, func):
		for key, value in self.plots.items():
			ani = animation.FuncAnimation(self.fig, func, fargs=(value.x, value.y), interval=1000)