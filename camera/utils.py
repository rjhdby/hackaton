def debug(func):
    def wrapper(*args, **kwargs):
        try:
            print(f"start function: {func}")
            func(*args, **kwargs)
            print(f"finish function: {func}")
        except Exception as e:
            print(e)
    return wrapper
