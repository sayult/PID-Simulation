import  matplotlib
from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
matplotlib.use('TkAgg')
import re
import subprocess
from matplotlib.widgets import Slider
import  os
import sys
import numpy as np
from numpy.ma.core import append
import tkinter as tk
from tkinter import ttk,font

def get_c_exe_path():
    if getattr(sys, 'frozen', False):
        base_path = sys._MEIPASS
    else:
        base_path = os.path.abspath(os.path.abspath(__file__))
    c_exe_name = "pid_simulation.exe" if os.name == 'nt' else "pid_simulation"
    exe_path = os.path.join(base_path, c_exe_name)
    if not os.path.exists(exe_path):
        raise FileNotFoundError(f"PID仿真程序未找到：{exe_path}")
    return exe_path
C_EXE_PATH = get_c_exe_path()


def get_data(Kp:float,Ki:float,Kd:float)-> tuple[list[float],list[float],float,float]:
    overshoot,time_settle = 0.0,0.0
    try:
        startupinfo = subprocess.STARTUPINFO()
        startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startupinfo.wShowWindow = subprocess.SW_HIDE
        res = subprocess.run(
            [C_EXE_PATH,str(Kp),str(Ki),str(Kd)],
            capture_output=True,text=True,check=True,
            startupinfo=startupinfo,
            creationflags=subprocess.CREATE_NO_WINDOW,
            shell=False,
        )
        times,outputs = [],[]
        for line in res.stdout.strip().split("\n"):
            line = line.strip()
            if not line: continue

            if line.startswith("overshoot:"):
                try:
                    parts = re.split(r'\s+',line)
                    overshoot = float(parts[1].replace("%",""))
                except:
                    overshoot = 0.0
                continue

            if line.startswith("time_settle:"):
                try:
                    parts = [p for p in re.split(r'\s+',line) if p]
                    if len(parts) >= 2:
                        time_settle_str = parts[1]
                        time_settle = float(time_settle_str)
                    else:
                        time_settle = 0.0
                except Exception as e:
                    time_settle = 0.0
                continue



            parts=re.split(r"\s+",line)
            if len(line.split()) == 2:
                try:
                    t = float(line.split()[0])
                    out = float(line.split()[1])
                    times.append(t)
                    outputs.append(out)
                except ValueError:
                    continue
        return times,outputs,overshoot,time_settle
    except subprocess.CalledProcessError as e:
        print(f"仿真程序执行失败！：{e.stderr}")
        return [],[],0.0,0.0
    except Exception as e:
        print(f"数据获取失败：{str(e)}")
        return [],[],0.0,0.0

#主UI
class PIDSimulationUI:
    def __init__(self,root):
        self.root=root
        self.root.title("PID Simulation Application")

        self.root.iconbitmap("C:\\Users\\SAYULT\\Desktop\\C_PID\\favicon.ico")

        self.root.geometry("1000x1200")
        self.root.resizable(True,True)
        self.default_font = font.Font(family="微软雅黑",size=10)
        self.title_font = font.Font(family="微软雅黑",size=12,weight="bold")
        ttk.Style().configure('.',font=self.default_font)
        #初始化PID参数
        self.Kp_val=tk.DoubleVar(value=0.0)
        self.Ki_val=tk.DoubleVar(value=0.0)
        self.Kd_val=tk.DoubleVar(value=0.0)

        self.Kp = tk.StringVar(value="0.000")
        self.Ki = tk.StringVar(value="0.000")
        self.Kd = tk.StringVar(value="0.000")

        self.Kp_val.trace_add("write", self.sync_kp_var)
        self.Ki_val.trace_add("write", self.sync_ki_var)
        self.Kd_val.trace_add("write", self.sync_kd_var)

        self.Kp_val.trace_add("write",self.on_param_change)
        self.Ki_val.trace_add("write",self.on_param_change)
        self.Kd_val.trace_add("write", self.on_param_change)

        self.fig,self.ax = plt.subplots(figsize=(10,5))
        self.ax.set_xlabel("Time(s)",fontsize=10)
        self.ax.set_ylabel("Output Value",fontsize=10)
        self.ax.set_title("PID Simulation Application",fontsize=12,fontweight="bold")
        self.ax.grid(alpha=0.3,linestyle="--")
        self.line,=self.ax.plot([],[],color="#2E86AB",linewidth=2)

        self.canvas=FigureCanvasTkAgg(self.fig,master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH,expand=True,padx=10,pady=10)

        self.ctrl_frame=ttk.LabelFrame(self.root,text="PID Parameter",padding=10)
        self.ctrl_frame.pack(fill=tk.X,padx=10,pady=(0,10))

        self._crate_param_row(
            frame=self.ctrl_frame,
            label_text = "Kp:",
            var = self.Kp,
            val_var = self.Kp_val,
            min_val=0.0,
            max_val=2.0,
            resolution=0.01,
            row=0
        )

        self._crate_param_row(
            frame=self.ctrl_frame,
            label_text="Ki:",
            var=self.Ki,
            val_var=self.Ki_val,
            min_val=0.0,
            max_val=1.0,
            resolution=0.001,
            row=1
        )

        self._crate_param_row(
            frame=self.ctrl_frame,
            label_text="Kd:",
            var=self.Kd,
            val_var=self.Kd_val,
            min_val=0.0,
            max_val=2.0,
            resolution=0.01,
            row=2
        )

        self.perf_frame = ttk.LabelFrame(self.root, text="PID real-time indicators",padding=10)
        self.perf_frame.pack(fill=tk.X,padx = 10, pady = (0,10))

        self.dev_frame = ttk.LabelFrame(self.root, text = "Developer Info", padding=10)
        self.dev_frame.pack(fill=tk.X,padx = 10,pady = (0,10))
        dev_info_text = "Developer:Salyut_Skywalker | Version 1.2.7 20260104.6 | Contact: 2948174604@qq.com "
        self.dev_label = ttk.Label(
            self.dev_frame,
            text = dev_info_text,
            font = font.Font(family="微软雅黑",size = 9),
            anchor="center"
        )
        self.dev_label.pack(pady=5)

        ttk.Label(self.perf_frame, text = "Overshoot:",width = 15, anchor= "w").grid(row=0,column=0,padx=5,pady=5,sticky="w")
        self.overshoot_label = ttk.Label(self.perf_frame, text="0.00%", width=20, anchor="w")
        self.overshoot_label.grid(row=0,column=1,padx=5,pady=5)

        ttk.Label(self.perf_frame,text="settle time:",width=15,anchor="w").grid(row=0,column=2,padx=5,pady=5,sticky="w")
        self.time_settle_label = ttk.Label(self.perf_frame, text = "0.00s", width=20, anchor="w")
        self.time_settle_label.grid(row=0,column=3,padx=5,pady=5)
        self.perf_frame.grid_columnconfigure(1,weight=1)
        self.perf_frame.grid_columnconfigure(3,weight=1)

        self.update_plot()
    def _crate_param_row(self,frame,label_text,var,min_val,max_val,row,resolution,val_var):

            lbl = ttk.Label(frame, text=label_text, width=15, anchor="w")
            lbl.grid(row=row,column=0,padx=5,pady=5,sticky="w")

            slider = ttk.Scale(
                frame,
                from_=min_val,
                to=max_val,
                variable=val_var,
                orient = tk.HORIZONTAL,
                length=400,
                command=lambda v:self.on_slider_move()
            )

            slider.grid(row=row,column=1,padx=5,pady=5,sticky="we")
            entry = ttk.Entry(
                frame,
                textvariable=var,
                width=12,
                justify=tk.RIGHT,
            )

            entry.config(validate="focusout", validatecommand=(entry.register(lambda: True), '%P'))
            entry.bind("<FocusOut>",lambda e, s_var=var,d_var = val_var, min_v=min_val, max_v=max_val:self.validate_input(s_var,d_var,min_v,max_v))
            entry.grid(row=row,column=2,padx=5,pady=5)
            frame.grid_columnconfigure(1, weight=1)
    def on_slider_move(self,var):
                self.update_plot()
    def sync_kp_var(self, *args):
        self.Kp.set(f"{self.Kp_val.get():.3f}")
    def sync_ki_var(self, *agrs):
        self.Ki.set(f"{self.Ki_val.get():.3f}")
    def sync_kd_var(self, *agrs):
        self.Kd.set(f"{self.Kd_val.get():.3f}")

    def validate_input(self,var,min_val,max_val,d_var,s_var):
                try:
                    val = float(var.get())
                    val = round(val,3)
                    if val < min_val:
                        var.set(min_val)
                    elif val > max_val:
                        var.set(max_val)
                    val.set(round(val,3))
                    d_var.set(val)
                    s_var.set(f"{val:.3f}")

                except ValueError:
                    var.set(0.0)
                    s_var.set(0.000)
                self.update_plot()
    def on_param_change(self,*args):
        self.update_plot()

    def update_plot(self):

                Kp = self.Kp_val.get()
                Ki = self.Ki_val.get()
                Kd = self.Kd_val.get()

                times,outputs,overshoot,time_settle = get_data(Kp,Ki,Kd)

                if times and outputs:
                    self.line.set_xdata(times)
                    self.line.set_ydata(outputs)
                    self.ax.relim()
                    self.ax.autoscale_view()
                    self.overshoot_label.config(text=f"{overshoot:.2f}%")
                    self.time_settle_label.config(text=f"{time_settle:.2f}s")
                else:
                    self.line.set_xdata([])
                    self.line.set_ydata([])
                    self.overshoot_label.config(text="0.00%")
                    self.time_settle_label.config(text="0.00s")

                self.canvas.draw()

if __name__ == "__main__":
    root = tk.Tk()
    app = PIDSimulationUI(root)
    root.mainloop()











