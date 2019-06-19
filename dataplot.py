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
		self.plots[name] = {'title': title, 'ax': ax, 'x': x, 'y': y, 'plot': None}

	def get_plots(self):
		return self.plots

	def update_data(self, name, data):
		if self.plots[name]['plot'] is None:
			self.plots[name]['plot'] = self.plots[name]['ax'].plot(datetime.datetime.now().strftime('%H:%M:%S:%f'), [data])
			return
		self.plots[name]['x'].append(datetime.datetime.now().strftime('%H:%M:%S:%f'))
		self.plots[name]['y'].append(data)
		self.plots[name]['x'] = self.plots[name]['x'][-20:]
		self.plots[name]['y'] = self.plots[name]['y'][-20:]
		self.plots[name]['plot'].set_data(self.plots[name]['x'], self.plots[name]['y'])
		# ax.clear()

	def animate_helper(self):
		for key, value in self.plots.items():
			value['plot'].set_data(x.append(datetime.datetime.now().strftime('%H:%M:%S:%f')), value['datasource'])
			value['x'] = value['x'][-20:]
			value['y'] = value['y'][-20:]
			value['ax'].clear()


		print('hi')

	def animate(self, func):
		ani = animation.FuncAnimation(self.fig, self.animate_helper(), interval=20)