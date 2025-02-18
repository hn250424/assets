from renderer import Renderer
from settings import get_settings, set_settings, Settings

def main():
    # render.
    renderer = Renderer()
    renderer.render()

    # event.
    renderer.one_sendingButton.bind('<Button-1>', lambda e: test(renderer))

    # mainloop.
    renderer.run()

def test(renderer: Renderer):
    
    data = get_settings()
    v = renderer.one_elementSpinbox.get()
    data.one['0'].oneElement = v
    
    set_settings(data)
    

if __name__ == "__main__":
    main()