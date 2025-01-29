import tkinter
from typing import Literal


h_spinbox_button_width = 15
h_spinbox_button_font = ('Arial', 5)


class Spinbox(tkinter.Frame):
    '''
    Spinbox handling numeric types.
    '''

    def __init__(self, 
                 parent_frame,
                 value=None, 
                 from_=0, to=100, increment=1, data_type=float,
                 format='%.1f',
                 fieldbackground='white',  # 엔트리 배경.
                 foreground='black',       # 엔트리 폰트.
                 background='white',       # 버튼 배경.
                 arrowcolor='black',       # 버튼 폰트.
                 font=('Arial', 10),
                 **kwargs):
        super().__init__(parent_frame, **kwargs)


        # init.
        self.from_ = from_
        self.to = to
        self.increment = increment
        self.data_type = data_type
        self.format = format
        initial_value = self.format_value(value) if value is not None else self.format_value(from_)
        self.value = tkinter.StringVar(value=initial_value)  # 변수 self.value 타입은 StringVar. StringVar 안의 value 파라미터는 StringVar 객체를 초기화하는 값.
        self.is_readonly = False
        

        ''' self == spinbox == frame '''
        validate_cmd = self.register(self.validate_input)  # 숫자만 입력받아야 한다. 
        self.entry = tkinter.Entry(self, textvariable=self.value, validate='key', validatecommand=(validate_cmd, '%P'), bg=fieldbackground, fg=foreground, font=font)  # textvariable=self.value -> 유저가 입력한 엔트리 위젯 안의 텍스트를 self.value와 연동.
        self.button_frame = tkinter.Frame(self, bg=background)

        self.entry.grid(row=0, column=0, sticky='nsew')
        self.button_frame.grid(row=0, column=1, sticky='nsew')

        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, minsize=h_spinbox_button_width)

        self.grid_rowconfigure(0, weight=1)

        self.grid_propagate(False)


        ''' button frame '''
        self.up_button = tkinter.Button(self.button_frame, text='▲', command=self.increase_value, bg=background, fg=arrowcolor, font=h_spinbox_button_font)
        self.down_button = tkinter.Button(self.button_frame, text='▼', command=self.decrease_value, bg=background, fg=arrowcolor, font=h_spinbox_button_font)

        self.up_button.grid(row=0, column=0, sticky='nsew')
        self.down_button.grid(row=1, column=0, sticky='nsew')

        self.button_frame.grid_rowconfigure(0, weight=1)

        self.button_frame.grid_rowconfigure(1, weight=1)
        self.button_frame.grid_columnconfigure(0, weight=1)

        self.button_frame.grid_propagate(False)


        ''' bind '''
        self.entry.bind("<FocusOut>", self.on_entry_focus_out)


    def format_value(self, value):
        if(self.data_type == int):
            return str(value)  # int면 단순히 문자열로만 포맷.
        elif(self.data_type == float):
            return self.format % value  # Python 문자열 포맷팅 연산자. format에 지정된 형식에 따라 value를 문자열로 변환.


    def validate_input(self, input_value):
        ''' 일차적으로 유효한 값만 입력받아 유저의 혼란을 최소화. '''

        if self.data_type == int:  # 빈 문자열과 자연수만 허용.
            return input_value.isdigit() or input_value == ''
        elif self.data_type == float:  # 빈 문자열과 flaot으로 파싱이 가능한 문자열만 허용.
            if input_value == '':
                return True
            try:
                float(input_value)
                return True
            except ValueError:
                return False
        return False


    def validate_value(self):
        '''
        값이 쓰이기 전 최종적으로 유효한 값인지 검증. 유저가 공백이나 범위를 벗어난 유효하지 않은 값을 입력하고 포커스 아웃하지 않은 채 전송 버튼을 눌렀을 때와 같은 경우를 대비. 
        현재 엔트리 위젯에서 값을 가져와 유효한 값으로 변환한 넘버 타입 리턴.
        '''
        
        current = self.from_

        try:
            current = self.data_type(self.value.get())
            if current > self.to:
                current = self.to
            elif current < self.from_:
                current = self.from_
        except ValueError:
            current = self.from_

        return self.data_type(current)


    def increase_value(self):
        if self.is_readonly:
            return
        current = self.get()
        new_value = min(current + self.increment, self.to)
        self.value.set(self.format_value(new_value))


    def decrease_value(self):
        if self.is_readonly:
            return
        current = self.get()
        new_value = max(current - self.increment, self.from_)
        self.value.set(self.format_value(new_value))
            

    def set(self, text):
        self.entry.delete(0, 'end')
        self.entry.insert(0, text)


    def get(self):
        return self.validate_value()


    def on_entry_focus_out(self, event=None):
        '''
        It performs input validation and updates value when the entry widget loses focus.
        '''
        
        self.set(self.format_value(self.validate_value()))  # Ensure the displayed value is formatted


    def config(self, state: Literal['normal', 'readonly'] = 'normal'):
        if state == 'normal':
            self.entry.config(state='normal')
            self.is_readonly = False
        elif state == 'readonly':
            self.entry.config(state='readonly')
            self.is_readonly = True    