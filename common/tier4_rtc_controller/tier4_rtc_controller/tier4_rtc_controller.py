import tkinter
from tkinter import ttk

from rclpy.node import Node


class tier4_rtc_controller_node(Node):
    def __init__(self) -> None:
        super().__init__("tier4_rtc_controller")


def ros_main(args=None):
    root = tkinter.Tk()
    root.title("RTC Controller")
    root.geometry("300x200")
    radio_button = ttk.Radiobutton(
        root, text="Test", value=0, variable=tkinter.IntVar().set(0), state=tkinter.DISABLED
    )
    radio_button.place(x=70, y=40)
    root.mainloop()


if __name__ == "__main__":
    ros_main()
