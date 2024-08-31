from socket_com import Server

def ciao(mex):
    print("ciao")
    print(mex)

if __name__ == "__main__":
    server = Server(ciao)
