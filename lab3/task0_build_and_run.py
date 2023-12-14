from Viewer.controller import Controller, SimpleViewer
# Ragdoll 布娃娃系统
def main():
    viewer = SimpleViewer(float_base = True, substep = 32)
    viewer.run()

if __name__ == '__main__':
    main()