# List double equal

## Description

Compare JSON key value to the value passed in parameter and returns true if equal.

## Parameters

| Parameter         | Description                                                            | Type          | Default         |
| ----------------- | ---------------------------------------------------------------------- | ------------- | --------------- |
| **key**           | JSON key where value is located, separate nested dictionary with **/** | str           | N/A (Mandatory) |
| **value**         | Value to which compare the JSON value                                  | list\[float\] | N/A (Mandatory) |
| **order_matters** | If true, will compare taking account of the order                      | bool          | true            |
