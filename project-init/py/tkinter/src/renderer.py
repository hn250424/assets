import tkinter as tk
from tkinter import ttk

from h_spinbox import H_Spinbox

hand_cursor = 'hand2'

background_main = 'white'
foreground_radio = 'red'
main_width = 1000
main_height = 500

select_frame_pad = 10
setting_frame_pad = 10
button_pad = 10

selectBox_width = 200
selectBox_height = 50
selectLabel_width = 200
selectLabel_height = 50
operationButton_width = 100
operationButton_height = 70
sendingButton_width = 100
sendingButton_height = 70
radioButton_width = 50

settingLabel_width = 200
settingLabel_height = 50
settingBox_width = 200
settingBox_height = 50

combobox_fieldbackground = 'white'
combobox_background = '#D3D3D3'
combobox_foreground = 'black'
combobox_arrowcolor = 'black'
spinbox_fieldbackground = combobox_fieldbackground
spinbox_background = combobox_background
spinbox_foreground = combobox_foreground
spinbox_arrowcolor = combobox_arrowcolor

operationButton_background = '#4285F4'
operationButton_foreground = 'white'
operationButton_activebackground = 'gray'
operationButton_activeforeground = 'white'

sendingButton_background = '#4285F4'
sendingButton_foreground = 'white'
sendingButton_activebackground = 'gray'
sendingButton_activeforeground = 'white'


class Renderer():
    def render(self):
        self.main_window = tk.Tk() 
        self.main_window.title('Control System')
        self.main_window.configure(bg=background_main)
        self.main_window.geometry(f'{main_width}x{main_height}')

        notebook = ttk.Notebook(self.main_window)
        
        one_frame = ttk.Frame(notebook)
        two_frame = ttk.Frame(notebook)

        self.render_one(one_frame)
        self.render_two(two_frame)

        notebook.add(one_frame, text="one")
        notebook.add(two_frame, text="two")

        notebook.pack(expand=True, fill='both')

    def run(self):
        self.main_window.mainloop()

    def render_one(self, one_frame):
        """
        one_frame > select, setting, operation
        select_frame > selector, scope
        setting_frame > element
        operation > on, off
        sending > send
        """

        """one_frame > select, setting"""
        one_selectFrame = tk.Frame(one_frame, bg='red')
        one_settingFrame = tk.Frame(one_frame, bg='blue')
        one_operationFrame = tk.Frame(one_frame, bg='yellow')
        one_sendingFrame = tk.Frame(one_frame, bg='orange')

        one_selectFrame.grid(row=0, column=0, sticky='nsew')
        one_settingFrame.grid(row=1, column=0, sticky='nsew')
        one_operationFrame.grid(row=2, column=0, sticky='nsew')
        one_sendingFrame.grid(row=3, column=0, sticky='nsew')

        one_frame.grid_rowconfigure(0, minsize=selectBox_height)
        one_frame.grid_rowconfigure(1, minsize=selectBox_height)
        one_frame.grid_rowconfigure(2, minsize=operationButton_height)
        one_frame.grid_rowconfigure(3, weight=1)
        one_frame.grid_columnconfigure(0, weight=1)
        one_frame.grid_propagate(False)

        """select_frame > selector, scope"""
        one_selectorFrame = tk.Frame(one_selectFrame, padx=select_frame_pad, pady=select_frame_pad)
        one_scopeFrame = tk.Frame(one_selectFrame, padx=select_frame_pad, pady=select_frame_pad)

        one_selectorFrame.grid(row=0, column=0, sticky='nsew')
        one_scopeFrame.grid(row=0, column=1, sticky='nsew')

        one_selectFrame.grid_columnconfigure(0, weight=1)
        one_selectFrame.grid_columnconfigure(1, weight=1)
        one_selectFrame.grid_rowconfigure(0, minsize=selectBox_height)
        one_selectFrame.grid_propagate(False)

        """selector > label, box"""
        one_selectLabel = tk.Label(one_selectorFrame, text='one', bg='purple')
        one_selectption = ['one 0', 'one 1', 'one 2', 'one 3', 'one 4', 'one 5']
        self.one_selectBox = ttk.Combobox(one_selectorFrame, values=one_selectption, state='readonly', cursor=hand_cursor, width=5, background='red')
        self.one_selectBox.set(one_selectption[0])

        one_selectLabel.grid(row=0, column=0, sticky='nsew')
        self.one_selectBox.grid(row=0, column=1, sticky='nsew')

        one_selectorFrame.grid_columnconfigure(0, minsize=selectLabel_width)
        one_selectorFrame.grid_columnconfigure(1, minsize=selectBox_width)
        one_selectorFrame.grid_rowconfigure(0, weight=1)
        one_selectorFrame.grid_propagate(False)

        """scope"""
        self.one_scope = tk.StringVar(value='ALL')
        space = tk.Frame(one_scopeFrame)
        self.one_radioAllButton = tk.Radiobutton(one_scopeFrame, text='전체 적용', variable=self.one_scope, value='ALL', bg=background_main, fg=foreground_radio, activebackground=background_main, highlightbackground=background_main, cursor=hand_cursor)
        self.one_radioOneButton = tk.Radiobutton(one_scopeFrame, text='단일 적용', variable=self.one_scope, value='ONE', bg=background_main, fg=foreground_radio, activebackground=background_main, highlightbackground=background_main, cursor=hand_cursor)
        
        space.grid(row=0, column=0, sticky='nsew')
        self.one_radioAllButton.grid(row=0, column=1, sticky='nsew')
        self.one_radioOneButton.grid(row=0, column=2, sticky='nsew')

        one_scopeFrame.grid_columnconfigure(0, weight=1)
        one_scopeFrame.grid_columnconfigure(1, minsize=radioButton_width)
        one_scopeFrame.grid_columnconfigure(2, minsize=radioButton_width)
        
        one_scopeFrame.grid_rowconfigure(0, weight=1)

        one_scopeFrame.grid_propagate(False)

        """setting_frame > element"""
        one_elementFrame = tk.Frame(one_settingFrame, padx=setting_frame_pad, pady=setting_frame_pad, bg='grey')

        one_elementFrame.grid(row=0, column=0, sticky='nsew')

        one_settingFrame.grid_rowconfigure(0, minsize=settingBox_height)
        one_settingFrame.grid_columnconfigure(0, weight=1)
        one_settingFrame.grid_propagate(False)

        """each setting frame > label, box"""
        """element"""
        one_elementLabel = tk.Label(one_elementFrame, text='element', bg='green')
        self.one_elementSpinbox = H_Spinbox(one_elementFrame, from_=0, to=100, data_type=int, fieldbackground=spinbox_fieldbackground, background=spinbox_background, foreground=spinbox_foreground, arrowcolor=spinbox_arrowcolor)

        one_elementLabel.grid(row=0, column=0, sticky='nsew')
        self.one_elementSpinbox.grid(row=0, column=1, sticky='nsew')

        one_elementFrame.grid_columnconfigure(0, minsize=settingLabel_width)
        one_elementFrame.grid_columnconfigure(1, minsize=settingBox_width)
        one_elementFrame.grid_rowconfigure(0, weight=1)
        one_elementFrame.grid_propagate(False)

        """operation > on, off"""
        on = tk.Button(one_operationFrame, text='ON', bg=operationButton_background, fg=operationButton_foreground, activebackground=operationButton_activebackground, activeforeground=operationButton_activeforeground, cursor=hand_cursor)
        off = tk.Button(one_operationFrame, text='OFF', bg=operationButton_background, fg=operationButton_foreground, activebackground=operationButton_activebackground, activeforeground=operationButton_activeforeground, cursor=hand_cursor)

        on.grid(row=0, column=0, sticky='nsew', padx=button_pad, pady=button_pad)
        off.grid(row=0, column=1, sticky='nsew', padx=button_pad, pady=button_pad)

        one_operationFrame.grid_columnconfigure(0, minsize=operationButton_width)
        one_operationFrame.grid_columnconfigure(1, minsize=operationButton_width)

        one_operationFrame.grid_rowconfigure(0, minsize=operationButton_height)

        one_operationFrame.grid_propagate(False)

        """sending > send"""
        self.one_sendingButton = tk.Button(one_sendingFrame, text='전송', bg=sendingButton_background, fg=sendingButton_foreground, activebackground=sendingButton_activebackground, activeforeground=sendingButton_activeforeground, cursor=hand_cursor)

        self.one_sendingButton.grid(row=0, column=0, sticky='nsew', padx=button_pad, pady=button_pad)

        one_sendingFrame.grid_columnconfigure(0, minsize=sendingButton_width)
        one_sendingFrame.grid_rowconfigure(0, minsize=sendingButton_height)
        one_sendingFrame.grid_propagate(False)

    def render_two(self, two_frame):
        """
        two_frame > select, setting
        select_frame > selector, scope
        setting_frame > id, element
        sending > send
        """

        """two_frame > select, setting"""
        two_selectFrame = tk.Frame(two_frame, bg='red')
        two_settingFrame = tk.Frame(two_frame, bg='blue')
        two_sendingFrame = tk.Frame(two_frame, bg='orange')

        two_selectFrame.grid(row=0, column=0, sticky='nsew')
        two_settingFrame.grid(row=1, column=0, sticky='nsew')
        two_sendingFrame.grid(row=2, column=0, sticky='nsew')

        two_frame.grid_rowconfigure(0, minsize=selectBox_height)
        two_frame.grid_rowconfigure(1, minsize=settingBox_height * 2)
        two_frame.grid_rowconfigure(2, weight=1)
        two_frame.grid_columnconfigure(0, weight=1)
        two_frame.grid_propagate(False)

        """select_frame > selector, scope"""
        two_selectWrapperFrame = tk.Frame(two_selectFrame, padx=select_frame_pad, pady=select_frame_pad)
        two_scopeFrame = tk.Frame(two_selectFrame, padx=select_frame_pad, pady=select_frame_pad)

        two_selectWrapperFrame.grid(row=0, column=0, sticky='nsew')
        two_scopeFrame.grid(row=0, column=1, sticky='nsew')

        two_selectFrame.grid_columnconfigure(0, weight=1)
        two_selectFrame.grid_columnconfigure(1, weight=1)
        two_selectFrame.grid_rowconfigure(0, minsize=selectBox_height)
        two_selectFrame.grid_propagate(False)

        """selector > label, box"""
        two_selectLabel = tk.Label(two_selectWrapperFrame, text='two', bg='purple')
        two_selectOption = ['two 0', 'two 1', 'two 2', 'two 3', 'two 4', 'two 5']
        self.two_selectBox = ttk.Combobox(two_selectWrapperFrame, values=two_selectOption, state='readonly', cursor=hand_cursor, width=5, background='red')
        self.two_selectBox.set(two_selectOption[0])

        two_selectLabel.grid(row=0, column=0, sticky='nsew')
        self.two_selectBox.grid(row=0, column=1, sticky='nsew')

        two_selectWrapperFrame.grid_columnconfigure(0, minsize=selectLabel_width)
        two_selectWrapperFrame.grid_columnconfigure(1, minsize=selectBox_width)
        two_selectWrapperFrame.grid_rowconfigure(0, weight=1)
        two_selectWrapperFrame.grid_propagate(False)

        """scope"""
        self.two_scope = tk.StringVar(value='ALL')
        space = tk.Frame(two_scopeFrame)
        self.two_radioAllButton = tk.Radiobutton(two_scopeFrame, text='전체 적용', variable=self.two_scope, value='ALL', bg=background_main, fg=foreground_radio, activebackground=background_main, highlightbackground=background_main, cursor=hand_cursor)
        self.two_radioOneButton = tk.Radiobutton(two_scopeFrame, text='단일 적용', variable=self.two_scope, value='ONE', bg=background_main, fg=foreground_radio, activebackground=background_main, highlightbackground=background_main, cursor=hand_cursor)
        
        space.grid(row=0, column=0, sticky='nsew')
        self.two_radioAllButton.grid(row=0, column=1, sticky='nsew')
        self.two_radioOneButton.grid(row=0, column=2, sticky='nsew')

        two_scopeFrame.grid_columnconfigure(0, weight=1)
        two_scopeFrame.grid_columnconfigure(1, minsize=radioButton_width)
        two_scopeFrame.grid_columnconfigure(2, minsize=radioButton_width)
        
        two_scopeFrame.grid_rowconfigure(0, weight=1)

        two_scopeFrame.grid_propagate(False)

        """setting_frame >id, elemnt"""
        two_idFrame = tk.Frame(two_settingFrame, padx=setting_frame_pad, pady=setting_frame_pad, bg='grey')
        two_elementFrame = tk.Frame(two_settingFrame, padx=setting_frame_pad, pady=setting_frame_pad, bg='yellow')

        two_idFrame.grid(row=0, column=0, sticky='nsew')
        two_elementFrame.grid(row=1, column=0, sticky='nsew')

        two_settingFrame.grid_rowconfigure(0, minsize=settingBox_height)
        two_settingFrame.grid_rowconfigure(1, minsize=settingBox_height)
        two_settingFrame.grid_columnconfigure(0, weight=1)
        two_settingFrame.grid_propagate(False)

        """each setting frame > label, box"""
        """id"""
        two_idLabel = tk.Label(two_idFrame, text='id', bg='green')
        two_idOption = ['A', 'B']
        self.two_idCombobox = ttk.Combobox(two_idFrame, values=two_idOption, state='readonly', cursor=hand_cursor, width=5, background='red')
        self.two_idCombobox.set(two_idOption[0])

        two_idLabel.grid(row=0, column=0, sticky='nsew')
        self.two_idCombobox.grid(row=0, column=1, sticky='nsew')

        two_idFrame.grid_columnconfigure(0, minsize=settingLabel_width)
        two_idFrame.grid_columnconfigure(1, minsize=settingBox_width)
        two_idFrame.grid_rowconfigure(0, weight=1)
        two_idFrame.grid_propagate(False)

        """element"""
        two_elementLabel = tk.Label(two_elementFrame, text='Pulse number', bg='green')
        self.two_elementSpinbox = H_Spinbox(two_elementFrame, from_=0, to=4294967296, data_type=int, fieldbackground=spinbox_fieldbackground, background=spinbox_background, foreground=spinbox_foreground, arrowcolor=spinbox_arrowcolor)

        two_elementLabel.grid(row=0, column=0, sticky='nsew')
        self.two_elementSpinbox.grid(row=0, column=1, sticky='nsew')

        two_elementFrame.grid_columnconfigure(0, minsize=settingLabel_width)
        two_elementFrame.grid_columnconfigure(1, minsize=settingBox_width)
        two_elementFrame.grid_rowconfigure(0, weight=1)
        two_elementFrame.grid_propagate(False)

        """sending > send"""
        self.two_sendingButton = tk.Button(two_sendingFrame, text='전송', bg=sendingButton_background, fg=sendingButton_foreground, activebackground=sendingButton_activebackground, activeforeground=sendingButton_activeforeground, cursor=hand_cursor)

        self.two_sendingButton.grid(row=0, column=0, sticky='nsew', padx=button_pad, pady=button_pad)

        two_sendingFrame.grid_columnconfigure(0, minsize=sendingButton_width)
        two_sendingFrame.grid_rowconfigure(0, minsize=sendingButton_height)
        two_sendingFrame.grid_propagate(False)









        

