from socket_com import Server
import toml

config = toml.load("config.toml")


if __name__ == "__main__":
    server = Server(config["remote_ros"])
