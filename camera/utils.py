def debug(func):
    def wrapper(*args, **kwargs):
        try:
            print(f"start function: {func}")
            result = func(*args, **kwargs)
            print(f"finish function: {func}")
            return result
        except Exception as e:
            print(e)
    return wrapper
