import tkinter as tk
from spinbox import Spinbox

view_width = 500
view_height = 400

spinbox_fieldbackground = 'grey'
spinbox_background = 'green'
spinbox_foreground = 'red'
spinbox_arrowcolor = 'black'
settingbox_font = 'Arial'

view = tk.Tk()
view.geometry(f'{view_width}x{view_height}')

frame = tk.Frame(view)
frame.grid(row=0, column=0, sticky='nsew', padx=10, pady=10)
view.grid_rowconfigure(0, weight=1)
view.grid_columnconfigure(0, weight=1)
view.grid_propagate(False)

sb = Spinbox(frame, from_=16, to=2048, increment=16, data_type=int)
sb.grid(row=0, column=0, sticky='nsew', padx=10, pady=10)
frame.grid_rowconfigure(0, minsize=50)
frame.grid_columnconfigure(0, weight=1)
frame.grid_propagate(False)

view.mainloop()


