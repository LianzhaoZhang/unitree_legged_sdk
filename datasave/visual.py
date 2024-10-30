import pandas as pd
import matplotlib.pyplot as plt

def plot_data():
    # 设置中文字体支持
    plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
    plt.rcParams['axes.unicode_minus'] = False    # 用来正常显示负号
    
    # 创建图形窗口
    fig = plt.figure(figsize=(15, 10))
    
    try:
        # 读取足端力数据
        force_data = pd.read_csv('/home/zlz/Downloads/UnitreeTest/unitree_legged_sdk/datasave/foot_force_standup_fsm.csv')
        
        # 创建足端力子图
        ax1 = plt.subplot(1, 1, 1)
        ax1.plot(force_data['Time'], force_data['RF_Force'], 'r-', label='RF Force', linewidth=2)
        ax1.plot(force_data['Time'], force_data['LF_Force'], 'g-', label='LF Force', linewidth=2)
        ax1.plot(force_data['Time'], force_data['RH_Force'], 'b-', label='RH Force', linewidth=2)
        ax1.plot(force_data['Time'], force_data['LH_Force'], 'y-', label='LH Force', linewidth=2)
        ax1.axhline(40, color='k', linestyle='--')  # 添加y=40的参考线
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Force')
        ax1.set_title('Foot Force Data')
        ax1.grid(True)
        ax1.legend()
        
        # # 读取姿态角数据
        # rpy_data = pd.read_csv('/home/zlz/quadruped_ws/dataSave/rpy_real_1.csv')
        
        # # 创建姿态角子图
        # ax2 = plt.subplot(2, 1, 2)
        # ax2.plot(rpy_data['Time'], rpy_data['Roll'], 'r-', label='Roll', linewidth=2)
        # ax2.plot(rpy_data['Time'], rpy_data['Pitch'], 'g-', label='Pitch', linewidth=2)
        # ax2.plot(rpy_data['Time'], rpy_data['Yaw'], 'b-', label='Yaw', linewidth=2)
        # ax2.set_xlabel('Time (s)')
        # ax2.set_ylabel('Angle (degrees)')
        # ax2.set_title('Robot Pose Data')
        # ax2.grid(True)
        # ax2.legend()
        
        # 调整子图间距
        plt.tight_layout()
        
        # 显示图像
        plt.show()
        
    except FileNotFoundError:
        print("错误：找不到数据文件，请确保数据文件存在于指定路径")
    except Exception as e:
        print(f"错误：{str(e)}")

def plotFootForce():
    # 设置中文字体支持
    plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
    plt.rcParams['axes.unicode_minus'] = False    # 用来正常显示负号
    
    # 创建图形窗口
    fig = plt.figure(figsize=(15, 10))
    
    try:
        # 读取足端力数据
        force_data = pd.read_csv('/home/zlz/Downloads/UnitreeTest/unitree_legged_sdk/datasave/foot_force_standup_liedown.csv')
        
        # 创建足端力子图1
        fig1, ax1 = plt.subplots(figsize=(15, 10))
        ax1.plot(force_data['Time'], force_data['RF_Force'], 'r-', label='RF Force', linewidth=2)
        ax1.axhline(40, color='k', linestyle='--')  # 添加y=40的参考线
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Force (N)')
        ax1.set_title('RF Force Data')
        ax1.grid(True)
        ax1.legend()

        # 创建足端力子图2
        fig2, ax2 = plt.subplots(figsize=(15, 10))
        ax2.plot(force_data['Time'], force_data['LF_Force'], 'g-', label='LF Force', linewidth=2)
        ax2.axhline(40, color='k', linestyle='--')  # 添加y=40的参考线
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Force (N)')
        ax2.set_title('LF Force Data')
        ax2.grid(True)
        ax2.legend()

        # 创建足端力子图3
        fig3, ax3 = plt.subplots(figsize=(15, 10))
        ax3.plot(force_data['Time'], force_data['RH_Force'], 'b-', label='RH Force', linewidth=2)
        ax3.axhline(40, color='k', linestyle='--')  # 添加y=40的参考线
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Force (N)')
        ax3.set_title('RH Force Data')
        ax3.grid(True)
        ax3.legend()

        # 创建足端力子图4
        fig4, ax4 = plt.subplots(figsize=(15, 10))
        ax4.plot(force_data['Time'], force_data['LH_Force'], 'y-', label='LH Force', linewidth=2)
        ax4.axhline(40, color='k', linestyle='--')  # 添加y=40的参考线
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Force (N)')
        ax4.set_title('LH Force Data')
        ax4.grid(True)
        ax4.legend()

        # 显示图像
        plt.show()
        
    except FileNotFoundError:
        print("错误：找不到数据文件，请确保数据文件存在于指定路径")
    except Exception as e:
        print(f"错误：{str(e)}")

if __name__ == "__main__":
    plot_data()