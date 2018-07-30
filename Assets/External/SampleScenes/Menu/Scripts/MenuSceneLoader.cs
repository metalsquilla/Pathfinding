using System;
using UnityEngine;

public class MenuSceneLoader : MonoBehaviour
{
    public UnityEngine.GameObject menuUI;

    private UnityEngine.GameObject m_Go;

	void Awake ()
	{
	    if (m_Go == null)
	    {
	        m_Go = Instantiate(menuUI);
	    }
	}
}
