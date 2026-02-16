# GDR Strings and Security Reference

## 1) String Formatting, When Functions Want to Format

If a function accepts `(msg, *args, **kwargs)`, pass the format string and arguments separately so the function can format.

```python
def error(msg, *args, **kwargs):
  ...
```

**Don't:**

```python
log_msg = 'failed to find insance ID: ' + str(instance_id)
logging.error(log_msg)
```

**Don't:**

```python
logging.error('failed to find instance ID: {}'.format(instance_id))
```

**Don't:**

```python
logging.error('failed to find instance ID: %s' % instance_id)
```

**Do:**

```python
logging.error('failed to find instance ID: %s', instance_id)
```

Why GDR recommends this pattern:

1. The function can decide whether to format at all. For example, high logging level means `logging.debug(...)` can skip formatting cost.
2. The function can do better work with unformatted params. Example, logger or Sentry can categorize repeated message templates.
3. The function can use separate arguments directly. A pre-formatted single string loses structure and may require parsing later.

## 2) General String Formatting Use Cases

Use the most suitable formatting style for context.

### f-string (preferred in Python 3)

Most efficient because formatting is more static.

```python
f'added {len(items)} items'
```

### str.format()

Improved version of %-format.

```python
'added {} items'.format(len(items))
```

### %-format

Old C-style format string.

```python
'added %d items' % len(items)
```

### When f-string cannot be used

If format string is dynamic, f-string cannot be used.

```python
i18n_msgs = load_i18n_msgs()
print(i18n_msgs['en']['hello'].format(name='Nancy'))
```

In this case, `i18n_msgs['en']['hello']` can be like `"Hello, {0}"`, so f-string is not applicable.

## 3) String Injection Vulnerabilities

Do not allow string injection vulnerabilities such as eval, SQL injection, shell injection.

### Do not use `eval()`

GDR guidance: do not use `eval()`. If the string is constant, write it directly in source instead. There should be no practical use case unless sanitization and sandboxing are guaranteed.

### Shell injection example

```python
os.system("/path/to/my_command {}".format(args.action))
```

What if `--action="status; sudo rm -rf /"` is passed?

### Mitigations

1. Use `subprocess` module instead of `os.system`.
2. Use a list of args instead of a single command string, for example `['/path/to/my_command', args.action]`.
3. Do not use `shell=True`.

Also keep SQL injection in mind, search for "SQL injection examples".
