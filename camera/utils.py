def debug(func):
    def wrapper(*args, **kwargs):
        print(f"start function: {func}")
        func(*args, **kwargs)
        print(f"finish function: {func}")
    return wrapper
