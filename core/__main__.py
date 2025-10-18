# core/__main__.py
from .HighLevel.EntryPoint.app import App

if __name__ == "__main__":
    myapp = App()
    myapp.run()
    myapp.exit()