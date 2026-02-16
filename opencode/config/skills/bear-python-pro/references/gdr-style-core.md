# GDR Style Core

This reference captures core rules from the GDR Python guidelines for style, imports, module naming, trailing commas, and variable assignment patterns.

## 1) Google Style Guide and Formatter Policy

- Follow the Google Python Style Guide: <https://google.github.io/styleguide/pyguide.html>
- Use Ruff as the formatter.
- Priority rule:
  1. If Ruff conflicts with Google Style Guide, follow Google Style Guide first, then format again.
  2. If Ruff remains incompatible, follow Ruff.
- Do not use any other formatter for this policy.

## 2) Django Project Import Format

For Django projects, for example Universe, follow Django import style:
<https://docs.djangoproject.com/en/dev/internals/contributing/writing-code/coding-style/#imports>

Use the Django import grouping and ordering pattern.

**Do (Django import ordering example):**

```python
import datetime

from django.conf import settings
from django.db import models

from myapp.models import Robot
from myapp.services import robot_service
```

## 3) Module Naming

The module name is a namespace, especially when following the Google convention to import modules instead of individual classes and functions.

This rule does not apply for Universe side.

**Don't:**

```python
data.data_fetcher
date.date_today
date.date_year
numpy.numpy_ndarray
```

**Do:**

```python
data.fetcher
date.today
date.year
numpy.ndarray
```

## 4) Trailing Commas in Sequences

Trailing commas improve consistency and reduce noisy diffs.

When adding a new final item without a trailing comma, more lines change and blame can point at the wrong commit:

```diff
--- before.py   2025-02-27 22:56:41.839071585 +0900
+++ after.py    2025-02-27 22:56:59.033072118 +0900
@@ -1,4 +1,5 @@
 my_list = [
     "hello",
-    "world"
+    "world",
+    "!"
 ]
```

With a trailing comma, the diff is cleaner and blame stays accurate:

```diff
--- before.py   2025-02-27 23:02:29.084082112 +0900
+++ after.py    2025-02-27 23:02:22.706081916 +0900
@@ -1,4 +1,5 @@
 my_list = [
     "hello",
     "world",
+    "!",
 ]
```

Trailing commas also help prevent accidental string literal concatenation bugs:

```diff
--- before.py   2025-02-27 23:02:29.084082112 +0900
+++ after.py    2025-02-27 23:02:22.706081916 +0900
@@ -1,4 +1,5 @@
 my_list = [
     "hello",
     "world"
+    "!"
 ]
```

The diff above can silently produce `my_list == ["hello", "world!"]` instead of three items.

Rule of thumb:

- If the closing bracket is on the same line as the last item, omit trailing comma.
- If the closing bracket is on a separate line, include trailing comma.

**Preferred for multi-line structures:**

```python
my_list = [
    "hello",
    "world",
]
```

**Avoid in multi-line structures:**

```python
# Bad
my_list = [
    "hello",
    "world"
]
```

**No trailing comma when closing bracket is on same line:**

```python
my_tuple = ("hello", "world")
```

Outdated formatter-specific examples are less relevant with Ruff. In practice, the simple bracket-position rule above is usually sufficient.

### Trailing commas not required

```python
## 1. When elements fit in one line.
simple_list = ['controller', 'organizer', 'systemer']
deliver(start='table1', goal='table7')

## 2. When the elements look aligned or each elements are short enough.
bear_msgs.msg.SoundRequest(text=sound_text,
                           gender=sound_gender,
                           language=sound_language)

## 3. When elements are obvious for most developers.
## In this case you can see that elements must be str type paths
## without taking a look into the function.
target_path = os.path.join(self._config['bos.config.path'],
                           '/path/to/target', target_file_name)
```

### Trailing commas highly recommended

```python
## 1. When each elements are too long.
hbd_sound = bear_msgs.msg.SoundRequest(
    text='Happy Birthday {}'.format(customer_name),
    gender=bear_msgs.msg.SoundRequest.GENDER_NEUTRAL,
    language=bear_msgs.msg.SoundRequest.LANGUAGE_ENGLISH,
)

## 2. When the readability is not very good.
## Elements below are not aligned at all.
network_list = {
    'networks': [
        {'signal_strength': 90, 'security_type': 'WPA2 WPA21',
         'ssid': 'xfinity', 'password': 'abc'},
        {'signal_strength': 72, 'security_type': '', 'ssid': 'wifi'},
        {'signal_strength': 64, 'security_type': 'WPA2', 'ssid': 'w1',
         'password': '1234'}
    ],
    'remembered_networks': remembered_networks,
}

## This one takes several more lines, but its readability got improved much better.
network_list = {
    'networks': [{
        'signal_strength': 90,
        'security_type': 'WPA2 WPA1',
        'ssid': 'xfinity',
        'password': 'abc',
    }, {
        'signal_strength': 72,
        'security_type': '',
        'ssid': 'wifi',
    }, {
        'signal_strength': 64,
        'security_type': 'WPA2',
        'ssid': 'w1',
        'password': '1234',
    }],
    'remembered_networks': remembered_networks,
}

## 3. When new elements will be added/some will be edited soon.
## With the last trailing comma, diff is cleaner (just added lines).
flag_names = [
  'enable_ota',
  'ota_download_timeout',
  'enable_robot_turnaround',
]

## 4. When the elements are in nested blocks.
{{{
   simple_funcion(input1, input2),
}}}
```

## 5) Variable Assignment Patterns (Nit)

`a = b = value` emphasizes equality.

`a, b = value, value` reads better when equal values are coincidental.

**Not ideal:**

```python
x1 = y1 = 1  # BAD(?)
x2, y2 = 2, 3
```

This emphasizes equality for `x1` and `y1`, but here that equality is only incidental.

**Better:**

```python
## Test case from (1,1) to (2,3)
x1, y1 = 1, 1
x2, y2 = 2, 3
```

**Use equality emphasis when intentional:**

```python
## set start and end to the same destination the pause command
start = end = dest
```
