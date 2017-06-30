from tkinter import *
import launch

def main():
    Launcher().mainloop()

class Launcher(Frame):
    def __init__(self):
        Frame.__init__(self)
        self.textvars = {}
        self.master.title('KSP Launch to Orbit')
        self.build_form()
        self.link_launcher()

    def link_launcher(self):
        self.mac = launch.ModularAscentControl('GUI Launch')
        self.mac.display_telemetry = self.display_telemetry
        self.mac.display_status = self.display_status

    def build_form(self):
        # add launch button
        self.btn_launch = Button(self.master, text='Launch', command=self.launch)
        self.btn_launch.grid(row=0, column=0, columnspan=4, sticky=W+E, padx=5, pady=5)
        # add telemetry widgets
        for r, columnpair in enumerate((
            ('Apoapsis', 'Periapsis'),
            ('Time to Apo', 'Time to Peri'),
            ('Altitude', 'Inclination'),
            ('Orbital Velocity', 'Vertical Speed'),
            ('Latitude', 'Longitude'),
            ('Dynamic Pressure', 'G-Force'))):

            col1, col2 = columnpair
            l = Label(self.master, text=col1 + ':', padx=10, pady=5)
            l.grid(row=(r + 1), column=0, sticky=W)
            t = StringVar()
            e = Entry(self.master, justify=RIGHT, textvariable=t)
            e.grid(row=(r + 1), column=1, sticky=E+W, padx=5)
            self.textvars[col1] = t
            l = Label(self.master, text=col2 + ':', padx=10, pady=5)
            l.grid(row=(r + 1), column=2, sticky=W)
            t = StringVar()
            e = Entry(self.master, justify=RIGHT, textvariable=t)
            e.grid(row=(r + 1), column=3, sticky=E+W, padx=5)
            self.textvars[col2] = t

        l = Label(self.master, text='Status:', padx=10)
        l.grid(row=8, column=0, sticky=W)
        t = StringVar()
        e = Entry(self.master, justify=LEFT, textvariable=t)
        e.grid(row=8, column=1, columnspan=3, sticky=E+W, padx=5, pady=5)
        self.textvars['Status'] = t

    def display_telemetry(self, t):
        f = '{:8,.0f}'
        f1 = '{:8,.1f}'
        self.textvars['Apoapsis'].set(f.format(t.apoapsis))
        self.textvars['Periapsis'].set(f.format(t.periapsis))
        self.textvars['Time to Apo'].set(f.format(t.time_to_apo))
        self.textvars['Time to Peri'].set(f.format(t.time_to_peri))
        self.textvars['Altitude'].set(f.format(t.altitude))
        self.textvars['Inclination'].set(f1.format(t.inclination))
        self.textvars['Orbital Velocity'].set(f.format(t.velocity))
        self.textvars['Vertical Speed'].set(f.format(t.vertical_speed))
        self.textvars['Latitude'].set(f1.format(t.lat))
        self.textvars['Longitude'].set(f1.format(t.lon))
        self.textvars['Dynamic Pressure'].set(f.format(t.q))
        self.textvars['G-Force'].set(f1.format(t.g))
        self.update()

    def display_status(self, s):
        self.textvars['Status'].set(s)
        self.update()


    def launch(self):
        self.btn_launch.config(state=DISABLED)
        self.mac.to_orbit()

if __name__ == '__main__':
    main()
