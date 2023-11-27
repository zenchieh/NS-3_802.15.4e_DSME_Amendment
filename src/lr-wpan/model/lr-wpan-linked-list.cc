#include "lr-wpan-linked-list.h"

#include <iostream>

template <typename T>
LinkedList<T>::LinkedList()
{
    this->head = nullptr;
}

template <typename T>
void
LinkedList<T>::insertAtBeginning(T data)
{
    ListNode<T>* newNode = new ListNode<T>(data);
    newNode->next = head;
    head = newNode;
}

template <typename T>
void
LinkedList<T>::insertAtEnd(T data)
{
    ListNode<T>* newNode = new ListNode<T>(data);
    if (head == nullptr)
    {
        head = newNode;
        return;
    }
    
    ListNode<T>* temp = head;
    while (temp->next != nullptr)
    {
        temp = temp->next;
    }
    temp->next = newNode;
}

template <typename T>
void
LinkedList<T>::deleteAtBeginning()
{
    if (head == nullptr)
    {
        return;
    }

    ListNode<T>* temp = head;
    head = head->next;
    delete temp;
}

template <typename T>
void
LinkedList<T>::deleteAtEnd()
{
    if (head == nullptr)
    {
        return;
    }
    if (head->next == nullptr)
    {
        delete head;
        head = nullptr;
        return;
    }

    ListNode<T>* temp = head;
    while (temp->next->next != nullptr)
    {
        temp = temp->next;
    }
    delete temp->next;
    temp->next = nullptr;
}

template <typename T>
void
LinkedList<T>::printList()
{
    ListNode<T>* temp = head;
    std::cout << "element in linked list : "
    while (temp != nullptr)
    {
        std::cout << temp->data << " ";
        temp = temp->next;
    }
    std::cout << std::endl;
}
