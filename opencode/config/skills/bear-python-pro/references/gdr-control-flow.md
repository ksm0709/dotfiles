# GDR Control Flow Reference

This reference is based on GDR Python guidelines.

## 1) Avoid Iterating Using Indices

Using indices in a `for` loop is error-prone, and logic gets complicated.

**Don't:**

```python
for i in range(len(results)):
    # Do something with results[i]
```

**Do:**

```python
for result in results:
    # Do something with result
```

If index is also needed, use `enumerate`.

**Do:**

```python
for i, name in enumerate(names):
    # Do something with i and name.
```

If two lists are iterated at the same time, use `zip`.

**Do:**

```python
keys = ['key1', 'key2']
values = ['value1', 'value2']
for k, v in zip(keys, values):
    # Do something with k and v.
```

Adjacent pairs from the same list can also use `zip`.

**Do:**

```python
nums = [1, 4, 5, 9]
deltas = [b-a for a, b in zip(nums, nums[1:])]  # [3, 1, 4]
```

## 2) Reduce Unnecessary Mutations

Use literal values instead of step-by-step mutations.

**Don't:**

```python
data = {}
data['name'] = 'robot'
data['size'] = 5
return data
```

**Do:**

```python
return {
    'name': 'robot',
    'size': 5,
}
```

## 3) Avoid Local Boolean Variables

Local boolean variables make code harder to understand.

**Don't:**

```python
found = False
for page in pages:
    if page.name == name_to_find:
        found = True
        break
if not found:
    logger.error('not found')
```

Use `for-else` when suitable.

**Do:**

```python
for page in pages:
    if page.name == name_to_find:
        break
else:
    logger.error('not found')
```

For nested loops, avoid `found = True` plus multiple breaks. Extract a function that returns `True` or `False`.

**Don't:**

```python
found = False
for page in pages:
    for result in results:
        if result.keyword == keyword:
            found = True
            break
    if found:
        break
if found:
    # Do something
```

**Do:**

```python
def has_keyword_in_pages(pages, keyword):
    for page in pages:
        for result in results:
            if result.keyword == keyword:
                return True
    return False

if has_keyword_in_pages(pages, keyword):
    # Do something
```

## 4) Reduce Indentation

Keep normal flow unindented, and exceptional flow indented.

**Don't:**

```python
image = Image()
err = None
if location:
    m = fetch_map(location)
    image.location_name = location.name
    if m.active:
        success = image.load_from(m)
        if success:
            return image, err
        else:
            err = 'Map load failed'
            logging.error(err)
    else:
        err = 'Map is inactive'
        logger.error(err)
else:
    err = 'Location is not available'
    logger.error(err)
return image, err
```

Problems in this style:

1. Hard to follow normal flow.
2. Hard to know what is returned.
3. May return partially constructed objects.
4. Error is both logged and returned. It should be either handled/logged or bubbled up, not both.

Use guard clauses and early returns.

**Do:**

```python
if not location:
    return None, 'Location is not available'
m = fetch_map(location)
image = Image()
image.location_name = location.name
if not m.active:
    return None, 'Map is inactive'
success = image.load_from(m)
if not success:
    return None, 'Map load failed'
return image, None
```

Read unindented lines to follow normal flow.

## 5) Unnecessary `elif` after `return`

If a branch already returns, following `elif` is unnecessary. Use another `if`.

**Don't:**

```python
if validation == PairingCodeValidation.INCORRECT:
    return A
elif validation == PairingCodeValidation.INVALID:
    return B
## ..rest of logics go here..
```

**Do:**

```python
if validation == PairingCodeValidation.INCORRECT:
    return A
if validation == PairingCodeValidation.INVALID:
    return B
## ..rest of logics go here..
```
