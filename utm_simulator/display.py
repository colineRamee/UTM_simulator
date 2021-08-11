import tkinter as tk
from queue import Empty
import json
from PIL import ImageGrab


class Symbol():
    def __init__(self,display, x, y, radius, status, ownship, multiple_planning_agents, switch_to_reactive=False):
        self.display = display
        self.switch_to_reactive = switch_to_reactive
        if multiple_planning_agents:
            self.radius = radius / 2
        else:
            self.radius = radius
        _radius = self.display.meter2pix(self.radius)
        x_ = self.display.meter2pix_coords(x)
        y_ = self.display.meter2pix_coords(y)
        self.ownship = ownship
        if ownship:
            self.icon = self.display.canvas.create_oval(x_ - _radius, y_ - _radius, x_ + _radius, y_ + _radius)
            if self.switch_to_reactive:
                self.color='yellow'
            else:
                self.color='green'
        else:
            self.icon = self.display.canvas.create_oval(x_ - _radius, y_ - _radius, x_ + _radius, y_ + _radius)
            self.color='black'
        if status == 'boom':
            self.color = 'red'
        self.change_color()

    def delete(self):
        self.display.canvas.delete(self.icon)

    def move(self, x, y, status, switch_to_reactive=False):
        x_ = self.display.meter2pix_coords(x)
        y_ = self.display.meter2pix_coords(y)
        _radius = self.display.meter2pix(self.radius)
        if self.ownship:
            self.display.canvas.coords(self.icon, x_ - _radius, y_ - _radius, x_ + _radius, y_ + _radius)
        else:
            self.display.canvas.coords(self.icon, x_ - _radius, y_ - _radius, x_ + _radius, y_ + _radius)
        if status == 'boom' and self.color != 'red':
            self.color = 'red'
            self.change_color()
        elif status == 'ok':
            if self.ownship:
                if switch_to_reactive and self.color != 'yellow':
                    self.color = 'yellow'
                    self.change_color()
                elif not switch_to_reactive and self.color != 'green':
                    self.color = 'green'
                    self.change_color()
            elif not self.ownship and self.color != 'black':
                self.color = 'black'
                self.change_color()

    def change_color(self):
        self.display.canvas.itemconfig(self.icon, outline=self.color)


class Display():
    def __init__(self, update_queue, length_arena, multiple_planning_agents=True, display_update=200, static_obstacles=None):
        self.root = tk.Tk()
        self.root.title("Simulation")
        self.root.resizable(True, False)
        self.root.aspect(1,1,1,1)
        self.canvas = tk.Canvas(self.root, width=700, height=700, borderwidth=0, highlightthickness=0)
        self.canvas.pack()
        self.border_ratio = 0.1
        self.length=length_arena
        self.multiple_planning_agents=multiple_planning_agents
        self.update_queue = update_queue
        self.display_update=display_update  # in ms how often to show the new position
        self.symbols = {}
        self.canvas.bind("<Configure>",self.create_static_elements)
        self.static_obstacles = static_obstacles
        self.index_image = 0

    def create_static_elements(self, event):
        # Border
        x0=self.meter2pix_coords(0)
        y0=self.meter2pix_coords(0)
        x1=self.meter2pix_coords(self.length)
        y1=self.meter2pix_coords(self.length)
        self.canvas.create_line(x0,y0,x0,y1)
        self.canvas.create_line(x0, y0, x1, y0)
        self.canvas.create_line(x1, y1, x0, y1)
        self.canvas.create_line(x1, y1, x1, y0)
        # Static Obstacles
        if self.static_obstacles is not None:
            self.draw_static_obstacles()

    def draw_static_obstacles(self):
        with open(self.static_obstacles['parameters']) as f:
            data = json.load(f)
        for obstacle in data:
            x_ = self.meter2pix_coords(obstacle['position'][0])
            y_ = self.meter2pix_coords(obstacle['position'][1])
            _radius = self.meter2pix(obstacle['radius'])
            self.canvas.create_oval(x_ - _radius, y_ - _radius, x_ + _radius, y_ + _radius)

    def meter2pix(self,x):
        width_dis=self.canvas.winfo_width()
        return (1-self.border_ratio)*width_dis*x/self.length

    def meter2pix_coords(self,x):
        width_dis = self.canvas.winfo_width()
        return self.border_ratio*width_dis/2 + (1-self.border_ratio)*width_dis * x / self.length

    def save_image(self):
        filepath = 'video/test_image_'+str(self.index_image)+'.jpg'
        self.index_image += 1
        if self.index_image >= 200:
            x=self.root.winfo_rootx()+self.canvas.winfo_x()
            y=self.root.winfo_rooty()+self.canvas.winfo_y()
            x1=x+self.canvas.winfo_width()
            y1=y+self.canvas.winfo_height()
            ImageGrab.grab((x,y,x1,y1)).save(filepath)

    def update(self):
        try:
            update_agents = self.update_queue.get(timeout=10.0)
        except Empty:
            print('nothing in the queue')
            self.root.quit()
            return
        agents_to_delete = []
        for agent_id, symbol in self.symbols.items():
            # if an agent is not present in the list, delete its representation from the canvas
            # otherwise we just update its position
            if agent_id not in update_agents.keys():
                symbol.delete()
                agents_to_delete.append(agent_id)
            else:
                x = update_agents[agent_id]['x']
                y = update_agents[agent_id]['y']
                status = update_agents[agent_id]['status']
                switch_to_reactive = update_agents[agent_id]['switch_to_reactive']
                symbol.move(x, y, status, switch_to_reactive=switch_to_reactive)
        for agent_to_delete in agents_to_delete:
            self.symbols.pop(agent_to_delete)

        # If an agent did not exist before, create it
        for update_agent_id, update_agent in update_agents.items():
            if update_agent_id not in self.symbols:
                x = update_agent['x']
                y = update_agent['y']
                radius = update_agent['radius']
                status = update_agent['status']
                ownship = update_agent['ownship']
                switch_to_reactive = update_agent['switch_to_reactive']
                symbol = Symbol(self, x, y, radius, status, ownship, self.multiple_planning_agents, switch_to_reactive=switch_to_reactive)
                self.symbols[update_agent_id] = symbol

        self.save_image()
        self.canvas.after(self.display_update, self.update)

    def run(self):
        self.update()
        self.root.mainloop()


class LayeredDisplay:
    def __init__(self, n_displays, update_queue, length_arena, multiple_planning_agents=True, display_update=200, static_obstacles=None):
        self.root = tk.Tk()
        self.root.title("Display root")

        self.n_displays = n_displays
        self.update_queue = update_queue

        self.display_update = display_update

        self.canvases = []
        for i in range(0, n_displays):
            level = tk.Toplevel(self.root)
            level.resizable(True, False)
            level.aspect(1, 1, 1, 1)
            canvas = tk.Canvas(level, width=700, height=700, borderwidth=0, highlightthickness=0)
            canvas.pack()
            self.canvases.append(SubDisplay(canvas, length_arena,multiple_planning_agents=multiple_planning_agents, display_update=display_update, static_obstacles=static_obstacles))

    def run(self):
        self.update()
        self.root.mainloop()

    def update(self):
        try:
            update_agents = self.update_queue.get(timeout=10.0)
        except Empty:
            print('nothing in the queue')
            self.root.quit()
            return
        for i in range(0, self.n_displays):
            if i in update_agents:
                self.canvases[i].update(update_agents[i])
            else:
                self.canvases[i].update({})
        self.root.after(self.display_update, self.update)


class SubDisplay:
    def __init__(self, my_canvas, length_arena, multiple_planning_agents=True, display_update=200, static_obstacles=None):
        self.canvas=my_canvas
        self.symbols = {}
        self.border_ratio = 0.1
        self.length = length_arena
        self.multiple_planning_agents = multiple_planning_agents
        self.display_update = display_update
        self.static_obstacles = static_obstacles
        self.canvas.bind("<Configure>", self.create_static_elements)

    def create_static_elements(self, event):
        # Border
        x0 = self.meter2pix_coords(0)
        y0 = self.meter2pix_coords(0)
        x1 = self.meter2pix_coords(self.length)
        y1 = self.meter2pix_coords(self.length)
        self.canvas.create_line(x0, y0, x0, y1)
        self.canvas.create_line(x0, y0, x1, y0)
        self.canvas.create_line(x1, y1, x0, y1)
        self.canvas.create_line(x1, y1, x1, y0)
        # Static Obstacles
        if self.static_obstacles is not None:
            self.draw_static_obstacles()

    def draw_static_obstacles(self):
        with open(self.static_obstacles['parameters']) as f:
            data = json.load(f)
        for obstacle in data:
            x_ = self.meter2pix_coords(obstacle['position'][0])
            y_ = self.meter2pix_coords(obstacle['position'][1])
            _radius = self.meter2pix(obstacle['radius'])
            self.canvas.create_oval(x_ - _radius, y_ - _radius, x_ + _radius, y_ + _radius)

    def meter2pix(self, x):
        width_dis=self.canvas.winfo_width()
        return (1-self.border_ratio)*width_dis*x/self.length

    def meter2pix_coords(self,x):
        width_dis = self.canvas.winfo_width()
        return self.border_ratio*width_dis/2 + (1-self.border_ratio)*width_dis * x / self.length

    def update(self, update_agents):
        agents_to_delete = []
        for agent_id, symbol in self.symbols.items():
            # if an agent is not present in the list, delete its representation from the canvas
            # otherwise we just update its position
            if agent_id not in update_agents.keys():
                symbol.delete()
                agents_to_delete.append(agent_id)
            else:
                x = update_agents[agent_id]['x']
                y = update_agents[agent_id]['y']
                status = update_agents[agent_id]['status']
                switch_to_reactive = update_agents[agent_id]['switch_to_reactive']
                symbol.move(x, y, status, switch_to_reactive=switch_to_reactive)
        for agent_to_delete in agents_to_delete:
            self.symbols.pop(agent_to_delete)

        # If an agent did not exist before, create it
        for update_agent_id, update_agent in update_agents.items():
            if update_agent_id not in self.symbols:
                x = update_agent['x']
                y = update_agent['y']
                radius = update_agent['radius']
                status = update_agent['status']
                ownship = update_agent['ownship']
                switch_to_reactive = update_agent['switch_to_reactive']
                symbol = Symbol(self, x, y, radius, status, ownship, self.multiple_planning_agents, switch_to_reactive=switch_to_reactive)
                self.symbols[update_agent_id] = symbol

        #self.canvas.after(self.display_update, self.update)
